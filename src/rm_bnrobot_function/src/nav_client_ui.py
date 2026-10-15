#!/usr/bin/env python3
"""Qt desktop console for the shared navigation client; never opens CAN hardware."""
import math
import json
import os
import re
import sys
import time
from pathlib import Path
import numpy as np
import yaml
from PyQt5 import QtCore, QtGui, QtWidgets as W
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from rm_interfaces.msg import Goal, NavigationStatus
from rm_interfaces.srv import ConfigurePatrol
from ament_index_python.packages import get_package_share_directory


class MapView(W.QGraphicsView):
    selected = QtCore.pyqtSignal(float, float)
    def __init__(self):
        super().__init__();self.setScene(W.QGraphicsScene(self))
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setDragMode(W.QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush(QtGui.QColor('#101927'))
        self.message=None;self.map_item = None;self.overlays=[];self.robot=None;self.has_map=False
    def wheelEvent(self,event):
        self.setTransformationAnchor(W.QGraphicsView.AnchorUnderMouse)
        scale=1.15 if event.angleDelta().y()>0 else 1/1.15;self.scale(scale,scale)
    def mouseDoubleClickEvent(self,event):
        if self.has_map:
            p=self.mapToScene(event.pos())
            if self.map_item.contains(self.map_item.mapFromScene(p)):self.selected.emit(p.x(),-p.y())
        super().mouseDoubleClickEvent(event)
    def show_map(self,msg):
        if msg.header.frame_id!='map' or msg.info.width==0:return
        self.message=msg
        a=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        shade=np.where(a<0,72,np.where(a>=65,24,218)).astype(np.uint8)
        rgb=np.repeat(np.flipud(shade)[:,:,None],3,axis=2).copy()
        image=QtGui.QImage(rgb.data,msg.info.width,msg.info.height,msg.info.width*3,QtGui.QImage.Format_RGB888).copy()
        if self.map_item is None:self.map_item=self.scene().addPixmap(QtGui.QPixmap.fromImage(image))
        else:self.map_item.setPixmap(QtGui.QPixmap.fromImage(image))
        q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        transform=QtGui.QTransform();transform.translate(msg.info.origin.position.x,-msg.info.origin.position.y)
        transform.rotate(-math.degrees(yaw));transform.scale(msg.info.resolution,msg.info.resolution)
        transform.translate(0,-msg.info.height);self.map_item.setTransform(transform)
        self.map_item.setZValue(-10)
        if not self.has_map:self.has_map=True;self.fit_map()
    def value_at(self,x,y):
        if self.message is None:return None
        info=self.message.info;q=info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
        dx=x-info.origin.position.x;dy=y-info.origin.position.y
        col=math.floor((c*dx+s*dy)/info.resolution);row=math.floor((-s*dx+c*dy)/info.resolution)
        if not (0<=row<info.height and 0<=col<info.width):return -1
        return self.message.data[row*info.width+col]
    def fit_map(self):
        if self.map_item:self.fitInView(self.map_item.sceneBoundingRect().adjusted(-.5,-.5,.5,.5),QtCore.Qt.KeepAspectRatio)
    def show_points(self,points,connect=True):
        for item in self.overlays:self.scene().removeItem(item)
        self.overlays=[]
        pen=QtGui.QPen(QtGui.QColor('#35d5d0'),.04)
        for i,(name,x,y,z) in enumerate(points):
            if i and connect:
                prev=points[i-1];self.overlays.append(self.scene().addLine(prev[1],-prev[2],x,-y,pen))
            dot=self.scene().addEllipse(x-.1,-y-.1,.2,.2,pen,QtGui.QBrush(QtGui.QColor('#35d5d0')))
            text=self.scene().addSimpleText(f'{i+1} {name}');text.setBrush(QtGui.QColor('#008b87'));text.setFont(QtGui.QFont('DejaVu Sans',10));text.setFlag(W.QGraphicsItem.ItemIgnoresTransformations);text.setPos(x+.13,-y-.13)
            self.overlays.extend([dot,text])
    def show_robot(self,x,y,yaw):
        if self.robot:self.scene().removeItem(self.robot)
        polygon=QtGui.QPolygonF([QtCore.QPointF(.32,0),QtCore.QPointF(-.22,-.2),QtCore.QPointF(-.22,.2)])
        self.robot=self.scene().addPolygon(polygon,QtGui.QPen(QtGui.QColor('#1677ff'),.035),QtGui.QBrush(QtGui.QColor('#1677ff')))
        self.robot.setPos(x,-y);self.robot.setRotation(-math.degrees(yaw));self.robot.setZValue(5)


class NavigationConsole(W.QMainWindow):
    def __init__(self,node):
        super().__init__();self.node=node;self.last_status=0.;self.applied=False;self.busy=False;self.editor_revision=0;self.last_client_state=None
        self.setWindowTitle('巡检导航控制台 · Nav2');self.resize(1250,900)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,node)
        self.clients={name:node.create_client(Trigger,'/navigation/client/'+name) for name in ['start','pause','stop']}
        self.configure=node.create_client(ConfigurePatrol,'/navigation/client/configure')
        self.decision=node.create_client(ConfigurePatrol,'/navigation/decision/configure')
        self.emergency=node.create_publisher(Bool,'/emergency_stop',10)
        self.map_sub=node.create_subscription(OccupancyGrid,'/map',self.receive_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.status_sub=node.create_subscription(NavigationStatus,'/navigation/client/status',self.receive_status,10)
        self.spin_sub=node.create_subscription(Bool,'/spin_active',lambda m:self.spin_label.setText('受击自旋：执行中' if m.data else '受击自旋：未触发'),10)
        self.build_ui();self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(20)
        self.status_timer=QtCore.QTimer(self);self.status_timer.timeout.connect(self.refresh_pose);self.status_timer.start(200)
        default=node.declare_parameter('patrol_file',str(Path(get_package_share_directory('nav_plan'))/'config/patrol/custom_room_test.yaml')).value
        navigation_mode=node.declare_parameter('navigation_mode','prior').value
        if navigation_mode=='explore':
            self.mode.setCurrentIndex(self.mode.findData('explore'));self.file.clear();self.note('无先验地图模式：等待 SLAM 地图，目标由探索器自动生成')
        elif Path(default).is_file():self.load_file(default)
        self.explore_sub=node.create_subscription(String,'/exploration/status',self.receive_exploration,10)
    def build_ui(self):
        self.setStyleSheet('''QMainWindow,QWidget {background:#101927;color:#e1e9f4;font-family:"Noto Sans CJK SC","DejaVu Sans";font-size:13px;}
        QGroupBox {border:1px solid #304158;border-radius:10px;margin-top:18px;padding:12px;font-weight:bold;} QGroupBox::title {subcontrol-origin:margin;left:14px;padding:0 6px;}
        QPushButton {background:#243449;border:1px solid #3b506b;border-radius:6px;padding:9px 14px;} QPushButton:hover {background:#354c65;} QPushButton:disabled {color:#64758a;background:#182537;}
        QLineEdit,QDoubleSpinBox,QComboBox,QTableWidget {background:#172437;border:1px solid #304158;border-radius:5px;padding:5px;}
        QHeaderView::section {background:#243449;color:#bacbe0;padding:7px;border:0;} QTableWidget::item:selected {background:#215b72;} QTextEdit {background:#0b1320;border:1px solid #304158;}
        QCheckBox {padding:5px;}''')
        root=W.QWidget();self.setCentralWidget(root);layout=W.QVBoxLayout(root);layout.setContentsMargins(22,18,22,18);layout.setSpacing(14)
        header=W.QHBoxLayout();title=W.QLabel('巡检导航控制台');title.setStyleSheet('font-size:24px;font-weight:bold;');header.addWidget(title);header.addStretch()
        self.connected=W.QLabel('● 客户端未连接');header.addWidget(self.connected);layout.addLayout(header)
        cards=W.QHBoxLayout();self.state_label=W.QLabel('任务状态：等待连接');self.pose_label=W.QLabel('车体坐标：—');self.spin_label=W.QLabel('受击自旋：—')
        for label in [self.state_label,self.pose_label,self.spin_label]:label.setStyleSheet('background:#1b2a40;border-radius:8px;padding:14px;');cards.addWidget(label)
        layout.addLayout(cards)
        split=W.QSplitter();layout.addWidget(split,1)
        left=W.QWidget();ll=W.QVBoxLayout(left);ll.setContentsMargins(0,0,12,0)
        config=W.QGroupBox('任务与配置');form=W.QFormLayout(config)
        self.mode=W.QComboBox();self.mode.addItem('测试巡检 · 按列表执行','test');self.mode.addItem('正式决策 · 补给 / 占领','formal');self.mode.addItem('手动导航 · 单个目标','manual');self.mode.addItem('自动探索 · 无先验地图','explore');self.mode.addItem('探索到目标 · 逐段建图前往','explore_goal');form.addRow('运行方式',self.mode)
        self.file=W.QLineEdit();row=W.QHBoxLayout();row.addWidget(self.file);open_button=W.QPushButton('打开');open_button.clicked.connect(self.open_file);row.addWidget(open_button);form.addRow('目标文件',row)
        self.loop=W.QCheckBox('循环巡检');self.loop.setChecked(True);form.addRow(self.loop)
        self.wait=W.QDoubleSpinBox();self.wait.setRange(0,3600);self.wait.setValue(1);self.wait.setSuffix(' 秒');form.addRow('到点停留',self.wait)
        self.health=W.QDoubleSpinBox();self.health.setRange(.01,1);self.health.setSingleStep(.05);self.health.setValue(.25);form.addRow('补给血量阈值',self.health)
        ll.addWidget(config)
        group=W.QGroupBox('目标点 · 双击地图可填写坐标');gl=W.QVBoxLayout(group)
        self.table=W.QTableWidget(0,4);self.table.setMinimumHeight(170);self.table.setHorizontalHeaderLabels(['点名','x / m','y / m','z / m']);self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch);self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows);gl.addWidget(self.table)
        buttons=W.QHBoxLayout()
        for title,callback in [('添加',self.add_point),('删除',self.remove_point),('上移',lambda:self.move_point(-1)),('下移',lambda:self.move_point(1)),('另存为',self.save_file)]:
            b=W.QPushButton(title);b.clicked.connect(callback);buttons.addWidget(b)
        gl.addLayout(buttons);ll.addWidget(group,1)
        actions=W.QGridLayout();self.apply=W.QPushButton('应用配置（不启动）');self.apply.clicked.connect(self.apply_config);actions.addWidget(self.apply,0,0,1,2)
        self.start=W.QPushButton('开始 / 继续');self.start.setStyleSheet('background:#137f7a;');self.start.clicked.connect(lambda:self.control('start'));actions.addWidget(self.start,1,0)
        self.pause=W.QPushButton('暂停');self.pause.clicked.connect(lambda:self.control('pause'));actions.addWidget(self.pause,1,1)
        self.stop=W.QPushButton('取消任务');self.stop.clicked.connect(lambda:self.control('stop'));actions.addWidget(self.stop,2,0)
        release=W.QPushButton('解除急停');release.clicked.connect(lambda:self.set_emergency(False));actions.addWidget(release,2,1)
        emergency=W.QPushButton('急停 · 停止平移与自旋');emergency.setStyleSheet('background:#a52e40;font-weight:bold;');emergency.clicked.connect(lambda:self.set_emergency(True));actions.addWidget(emergency,3,0,1,2)
        ll.addLayout(actions);split.addWidget(left)
        right=W.QWidget();rl=W.QVBoxLayout(right);rl.setContentsMargins(0,0,0,0)
        map_header=W.QHBoxLayout();map_header.addWidget(W.QLabel('MAP / 实时地图与车体位置'));map_header.addStretch();fit=W.QPushButton('适应地图');fit.clicked.connect(lambda:self.map_view.fit_map());map_header.addWidget(fit);rl.addLayout(map_header)
        self.map_view=MapView();self.map_view.selected.connect(self.map_selected);rl.addWidget(self.map_view,1)
        self.target_label=W.QLabel('当前目标：—');rl.addWidget(self.target_label)
        self.explore_label=W.QLabel('');rl.addWidget(self.explore_label)
        self.log=W.QTextEdit();self.log.setReadOnly(True);self.log.setMaximumHeight(140);rl.addWidget(self.log);split.addWidget(right);split.setSizes([470,740])
        self.mode.currentIndexChanged.connect(self.changed);self.table.itemChanged.connect(self.changed);self.table.itemSelectionChanged.connect(lambda:self.changed() if self.mode.currentData()=="manual" else None);self.loop.toggled.connect(self.changed);self.wait.valueChanged.connect(self.changed);self.health.valueChanged.connect(self.changed)
        self.note('地图坐标单位为米。开始会应用当前修改并执行；单独应用配置会暂停任务。地图切换仍通过导航启动参数设置。')
    def note(self,text):self.log.append(time.strftime('%H:%M:%S')+'  '+text)
    def changed(self,*args):
        self.editor_revision+=1
        self.applied=False
        mode=self.mode.currentData();self.table.setEnabled(mode!='explore')
        self.loop.setEnabled(mode=='test');self.wait.setEnabled(mode=='test');self.health.setEnabled(mode=='formal')
        try:self.map_view.show_points([] if mode=='explore' else self.points())
        except ValueError:pass
    def points(self):
        points=[];names=set()
        for row in range(self.table.rowCount()):
            cells=[self.table.item(row,c).text().strip() if self.table.item(row,c) else '' for c in range(4)]
            name=cells[0]
            if not re.fullmatch(r'[A-Za-z0-9_]+',name) or name in names:raise ValueError('点名必须唯一，使用字母、数字或下划线')
            try:xyz=[float(v) for v in cells[1:]]
            except ValueError:raise ValueError('坐标必须是数字')
            if not all(math.isfinite(v) for v in xyz):raise ValueError('坐标必须是有限数值')
            points.append((name,*xyz));names.add(name)
        return points
    def add_point(self,checked=False,values=None):
        row=self.table.rowCount();self.table.insertRow(row)
        if values is None:
            existing={self.table.item(i,0).text() for i in range(row) if self.table.item(i,0)};index=row+1
            while f'p{index}' in existing:index+=1
            values=(f'p{index}',0.,0.,0.)
        for c,v in enumerate(values):self.table.setItem(row,c,W.QTableWidgetItem(str(v)))
        self.table.selectRow(row)
    def remove_point(self):
        row=self.table.currentRow()
        if row>=0:self.table.removeRow(row);self.changed()
    def move_point(self,delta):
        row=self.table.currentRow();other=row+delta
        if row<0 or not 0<=other<self.table.rowCount():return
        self.table.blockSignals(True)
        for col in range(4):
            a=self.table.takeItem(row,col);b=self.table.takeItem(other,col);self.table.setItem(row,col,b);self.table.setItem(other,col,a)
        self.table.blockSignals(False);self.table.selectRow(other);self.changed()
    def open_file(self):
        path,_=W.QFileDialog.getOpenFileName(self,'选择目标配置',self.file.text(),'YAML (*.yaml *.yml)')
        if path:self.load_file(path)
    def load_file(self,path):
        try:
            data=yaml.safe_load(Path(path).read_text());formal='nav_plan_node' in data
            params=data['nav_plan_node' if formal else 'nav_to_pose_client']['ros__parameters']
            if formal:
                points=[(name,*params['waypoints'][name]) for name in ['recovery','occupy','attack','defend']];self.health.setValue(float(params.get('health_threshold',.25)))
            else:
                patrol=params['patrol'];points=[(name,*patrol['points'][name]) for name in patrol['point_names']];self.loop.setChecked(bool(patrol.get('loop',True)));self.wait.setValue(float(patrol.get('wait_seconds',1.)))
            if not points or any(len(p)!=4 for p in points):raise ValueError('目标点必须包含 [x,y,z]')
            self.table.setRowCount(0)
            for point in points:self.add_point(values=point)
            self.points();self.mode.setCurrentIndex(self.mode.findData('formal' if formal else 'test'));self.file.setText(str(Path(path).resolve()));self.changed();self.note('已读取文件；尚未应用到机器人')
        except (OSError,KeyError,TypeError,ValueError,yaml.YAMLError) as exc:self.note('读取失败：'+str(exc))
    def save_file(self):
        try:
            points=self.points();mode=self.mode.currentData()
            if mode in ('explore','explore_goal'):raise ValueError('探索任务使用在线目标，自动探索参数位于 exploration.yaml')
            if not points:raise ValueError('请添加目标点')
            if mode=='formal':data={'nav_plan_node':{'ros__parameters':{'health_threshold':self.health.value(),'waypoints':{n:[x,y,z] for n,x,y,z in points}}}}
            else:data={'nav_to_pose_client':{'ros__parameters':{'goal.frame_id':'map','patrol':{'loop':self.loop.isChecked(),'wait_seconds':self.wait.value(),'point_names':[p[0] for p in points],'points':{n:[x,y,z] for n,x,y,z in points}}}}}
            path,_=W.QFileDialog.getSaveFileName(self,'保存目标配置',self.file.text() or 'patrol.yaml','YAML (*.yaml)')
            if path:Path(path).write_text(yaml.safe_dump(data,sort_keys=False,allow_unicode=True));self.file.setText(path);self.note('配置已保存到 '+path)
        except (OSError,ValueError) as exc:self.note('保存失败：'+str(exc))
    def request(self,client,request,done):
        if self.busy:self.note('正在处理上一条请求');return
        if not client.service_is_ready():self.note('服务未连接，请先启动导航客户端');return
        self.busy=True;future=client.call_async(request)
        deadline=time.monotonic()+5
        def poll():
            if future.done():
                timer.stop();timer.deleteLater();self.busy=False
                try:done(future.result())
                except Exception as exc:self.note('请求失败：'+str(exc))
            elif time.monotonic()>deadline:
                timer.stop();timer.deleteLater();self.busy=False;future.cancel();self.note('请求超时，请检查连接和当前状态')
        timer=QtCore.QTimer(self);timer.timeout.connect(poll);timer.start(30)
    def apply_config(self,checked=False,on_applied=None):
        try:
            mode=self.mode.currentData();points=[] if mode=='explore' else self.points()
            if not points and mode!='explore':raise ValueError('请添加目标点')
            if mode in ('explore','explore_goal') and 'frontier_explorer' not in self.node.get_node_names():raise ValueError('请以 navigation_mode:=explore 启动导航；该选择不会在线更换地图链路')
            if mode=='explore':points=[]
            if mode=='formal' and {p[0] for p in points}!={'recovery','occupy','attack','defend'}:raise ValueError('正式模式需要 recovery / occupy / attack / defend')
            if mode in ('manual','explore_goal'):
                row=self.table.currentRow()
                if row<0:raise ValueError('请选中一个手动目标')
                points=[points[row]]
                if mode=='manual' and self.map_view.value_at(points[0][1],points[0][2])==-1:
                    raise ValueError('目标尚未观测，请选择“探索到目标”逐段建图前往')
            req=ConfigurePatrol.Request();req.mode=mode;req.frame_id='map';req.loop=self.loop.isChecked();req.wait_seconds=self.wait.value();req.health_threshold=self.health.value()
            for name,x,y,z in points:
                p=Goal();p.goal_frame_id='map';p.goal_x=x;p.goal_y=y;p.goal_z=z;req.points.append(p);req.point_names.append(name)
            def apply_client(_=None):
                if _ is not None and not _.success:self.note(_.message);return
                self.request(self.configure,req,applied)
            revision=self.editor_revision
            def applied(res):
                self.applied=res.success and revision==self.editor_revision
                self.note(res.message if self.applied or not res.success else '配置已应用，但表格随后有修改，请重新点击开始或应用')
                if self.applied and on_applied is not None:on_applied()
            if mode=='formal':
                if not self.decision.service_is_ready():raise ValueError('正式决策未启动，请使用 navigation_mode:=prior patrol_mode:=formal 启动导航')
                # Pause the shared client before updating the decision source.
                self.request(self.clients['pause'],Trigger.Request(),lambda res:self.request(self.decision,req,apply_client) if res.success else self.note(res.message))
            else:apply_client()
        except ValueError as exc:self.note(str(exc))
    def control(self,name):
        if name=='start' and not self.applied:
            self.note('正在应用当前目标并开始导航')
            self.apply_config(on_applied=lambda:self.control('start'));return
        self.request(self.clients[name],Trigger.Request(),lambda res:self.note(res.message))
    def set_emergency(self,value):
        self.emergency.publish(Bool(data=value));self.note('已发布急停' if value else '已解除急停；不会自动恢复，请点击开始')
    def receive_exploration(self,msg):
        try:
            d=json.loads(msg.data)
            labels={'waiting_client':'等待客户端','waiting_map':'等待 SLAM 地图','waiting_motion':'等待底盘仲裁','waiting_costmap':'等待 Nav2 全局代价地图（未激活或无更新）','waiting_pose':'等待定位',
                'pose_out_of_map':'定位超出地图','inactive':'未启用','paused':'已暂停',
                'waiting_destination':'等待探索目的地','destination_reached':'探索目的地已到达','destination_blocked':'目的地位于障碍中，请修改目标','replanning':'正在换探索目标','navigating':'探索中','no_reachable_frontiers':'当前没有可达的探索前沿'}
            self.explore_label.setText('自动探索：'+labels.get(d['state'],d['state'])+
                f" · 已到达 {d['completed']} 个目标 · 失败 {d['failed']} 个")
            if self.mode.currentData() in ('explore','explore_goal') and d.get('goal'):
                points=[('next',*d['goal'],0.)]
                if d.get('destination'):points.append(('destination',*d['destination'],0.))
                self.map_view.show_points(points,connect=False)
        except (ValueError,KeyError,TypeError):pass
    def receive_map(self,msg):self.map_view.show_map(msg)
    def receive_status(self,msg):
        self.last_status=time.monotonic()
        labels={'waiting_mapping':'等待回环地图重建','waiting':'等待目标 / 停留','waiting_server':'等待 Nav2','waiting_motion':'等待底盘仲裁','waiting_localization':'等待定位','sending':'发送目标','executing':'导航中','complete':'已完成','failed':'导航失败','rejected':'目标被拒绝','paused':'已暂停','stopped':'已取消','ready':'配置就绪','emergency':'急停'}
        self.state_label.setText('任务状态：'+labels.get(msg.state,msg.state))
        self.state_label.setToolTip(msg.message)
        key=(msg.mode,msg.state,msg.point_name,msg.target.goal_x,msg.target.goal_y)
        if key!=self.last_client_state:
            self.last_client_state=key
            if msg.message:self.note(msg.message)
        distance=f'{msg.distance_remaining:.2f} m' if msg.distance_remaining>=0 else '—'
        p=msg.target;self.target_label.setText(f'当前目标：{msg.point_name or "决策目标"}  ({p.goal_x:.2f}, {p.goal_y:.2f})  ·  剩余 {distance}  ·  客户端模式 {msg.mode}')
    def tick(self):
        if rclpy.ok():
            deadline=time.monotonic()+.008
            for _ in range(200):
                rclpy.spin_once(self.node,timeout_sec=0.)
                if time.monotonic()>=deadline:break
    def refresh_pose(self):
        connected=time.monotonic()-self.last_status<2
        self.connected.setText('● 客户端已连接' if connected else '● 客户端未连接');self.connected.setStyleSheet('color:#35d5d0;' if connected else 'color:#fa9a72;')
        for button in [self.apply,self.start,self.pause,self.stop]:button.setEnabled(connected and not self.busy)
        try:
            t=self.buffer.lookup_transform('map','base_link',Time());p=t.transform.translation;q=t.transform.rotation
            age=self.node.get_clock().now().nanoseconds*1e-9-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
            if not -.05<=age<.5:raise ValueError('定位数据过期')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.pose_label.setText(f'车体坐标：{p.x:.2f}, {p.y:.2f} m');self.map_view.show_robot(p.x,p.y,yaw)
        except Exception:self.pose_label.setText('车体坐标：定位未就绪')
    def map_selected(self,x,y):
        mode=self.mode.currentData()
        if mode in ('explore','explore_goal','manual'):
            unknown=self.map_view.value_at(x,y)==-1
            if unknown and 'frontier_explorer' in self.node.get_node_names():
                self.mode.setCurrentIndex(self.mode.findData('explore_goal'))
                self.note('未知目的地：点击开始后探索可达前沿，逐段建图靠近目标')
            elif mode=='explore':
                self.mode.setCurrentIndex(self.mode.findData('manual'))
                self.note('已切换为手动目标；点击开始执行标点，在线建图继续。切回自动探索并点击开始可恢复探索')
        row=self.table.currentRow()
        if row<0:self.add_point();row=self.table.currentRow()
        for col,v in [(1,x),(2,y),(3,0.)]:self.table.setItem(row,col,W.QTableWidgetItem(f'{v:.3f}'))
        self.note(f'地图标点：x={x:.3f}, y={y:.3f} m；尚未发送')
    def closeEvent(self,event):
        self.timer.stop();self.status_timer.stop();event.accept()


def main():
    rclpy.init();app=W.QApplication(sys.argv);node=Node('nav_client_ui')
    window=NavigationConsole(node);window.show()
    quit_timer=QtCore.QTimer();quit_timer.timeout.connect(lambda:app.quit() if not rclpy.ok() else None);quit_timer.start(100)
    try:app.exec_()
    finally:window.close();node.destroy_node();rclpy.shutdown() if rclpy.ok() else None

if __name__=='__main__':main()
