import rclpy
from rclpy.node import Node

from sentry_interfaces.msg import SentryChassisPacket, Goal2D


class SentryDecisionNode(Node):
    """
    Sentry Decision Node (mm units)
    - Subscribe:  /chassis/packet (SentryChassisPacket)
    - Publish:    /motion/goal (Goal2D)
    Convention:
      Goal2D.x/y, ally_pos_x/y are treated as millimeters (mm).
    """

    def __init__(self):
        super().__init__("sentry_decision_node")

        # ========== Parameters ==========
        self.declare_parameter("goal_topic", "/motion/goal")
        self.declare_parameter("packet_topic", "/chassis/packet")
        self.declare_parameter("state", "demo")

        # which slot in ally_hp/ally_pos belongs to THIS robot (0/1/2)
        self.declare_parameter("self_index", 2)

        # supply triggers
        self.declare_parameter("hp_low", 80)            # hp < hp_low => force supply
        self.declare_parameter("zero_window_s", 30.0)   # hp==0 happened within window => force supply
        self.declare_parameter("hp_full", 400)  # 满血阈值：达到它才退出 supply
        self.hp_full = int(self.get_parameter("hp_full").value)

        self.in_supply = False  # supply 锁存状态

        # anti-spam: do not republish goal if moved < goal_eps (mm)
        self.declare_parameter("goal_eps", 50.0)

        # ========== Points (mm) ==========
        self.points = {
            "supply": (1000.0, 750.0),
            "occupy": (4000.0, 6000.0),

            "own_1": (3500.0, 1200.0),
            "own_2": (5000.0, 1200.0),
            "own_3": (6500.0, 1200.0),
            "own_tr": (6500.0, 3500.0),
            "own_oc": (6500.0, 6000.0),

            "opp_oc": (1500.0, 6000.0),
            "opp_tr": (1500.0, 12500.0),
            "opp_1": (4500.0, 14800.0),
            "opp_2": (3000.0, 14800.0),
            "opp_3": (1500.0, 14800.0),
        }

        # ========== Read parameters ==========
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.packet_topic = str(self.get_parameter("packet_topic").value)
        self.state = str(self.get_parameter("state").value)

        self.self_index = int(self.get_parameter("self_index").value)
        self.hp_low = int(self.get_parameter("hp_low").value)
        self.zero_window_s = float(self.get_parameter("zero_window_s").value)
        self.goal_eps = float(self.get_parameter("goal_eps").value)

        # clamp self_index once (still safe-checked later)
        self.self_index = max(0, min(2, self.self_index))

        # ========== IO ==========
        self.msg_latest = None
        self.sub = self.create_subscription(
            SentryChassisPacket,
            self.packet_topic,
            self.cb_packet,
            10
        )
        self.pub = self.create_publisher(Goal2D, self.goal_topic, 10)

        # ========== State / debounce ==========
        self.last_state = None
        self.last_goal = None  # (x,y) in mm
        self.last_zero_time_s = None  # float seconds (node clock)

        # main loop: 5 Hz
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info(
            f"Decision node started. packet={self.packet_topic}, goal={self.goal_topic}, "
            f"state={self.state}, self_index={self.self_index}, "
            f"hp_low={self.hp_low}, zero_window_s={self.zero_window_s}, goal_eps={self.goal_eps}mm"
        )

    # ---------- helpers ----------
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def get_self_hp(self, msg: SentryChassisPacket) -> int:
        idx = self.self_index
        # defensive check (even though self_index is clamped)
        if idx < 0 or idx >= len(msg.ally_hp):
            return 0
        return int(msg.ally_hp[idx])

    def publish_goal(self, x_mm: float, y_mm: float, valid: bool, goal_type: int):
        g = Goal2D()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.x = float(x_mm)
        g.y = float(y_mm)
        g.valid = bool(valid)
        g.goal_type = int(goal_type) & 0xFF
        self.pub.publish(g)

    # ---------- callbacks ----------
    def cb_packet(self, msg: SentryChassisPacket):
        self.msg_latest = msg

        self_hp = self.get_self_hp(msg)

        # record "hp==0 happened" timestamp
        if self_hp == 0:
            self.last_zero_time_s = self.now_s()

    # ---------- decision ----------
    def decide_goal(self, msg: SentryChassisPacket):
        self_hp = self.get_self_hp(msg)

    # ===== 进入 SUPPLY 锁存条件 =====
        if (not self.in_supply) and (self_hp < self.hp_low):
            self.in_supply = True
            self.get_logger().warn(f"ENTER SUPPLY: hp={self_hp} < {self.hp_low}")

    # ===== 退出 SUPPLY 锁存条件 =====
        if self.in_supply and (self_hp >= self.hp_full):
            self.in_supply = False
            self.get_logger().warn(f"EXIT  SUPPLY: hp={self_hp} >= {self.hp_full}")

    # ===== 锁存期间强制 supply =====
        if self.in_supply:
            x, y = self.points["supply"]
            return x, y, True, 2, f"SUPPLY(LATCH hp={self_hp})"

    # ===== 正常：按 state 点名直达（占位/联调）=====
        if self.state in self.points:
            x, y = self.points[self.state]
            return x, y, True, 1, f"PT:{self.state} hp={self_hp}"

        return 0.0, 0.0, False, 0, f"NO_STATE hp={self_hp}"

        # force supply
        if (self_hp < self.hp_low) or had_zero_recently:
            x, y = self.points["supply"]
            if self_hp < self.hp_low:
                reason = f"hp_low hp={self_hp}<{self.hp_low}"
            else:
                reason = f"zero_recent age={zero_age_s:.1f}s<= {self.zero_window_s:.1f}s"
            return x, y, True, 2, f"SUPPLY({reason})"

        # normal: go to named point by state
        if self.state in self.points:
            x, y = self.points[self.state]
            return x, y, True, 1, f"PT:{self.state}"

        # no valid state
        return 0.0, 0.0, False, 0, "NO_STATE"

    # ---------- loop ----------
    def loop(self):
        if self.msg_latest is None:
            self.publish_goal(0.0, 0.0, False, 0)
            self.get_logger().info("waiting packet... (publish invalid goal)")
            return

        gp = int(self.msg_latest.game_progress)
        if gp != 4:
            self.publish_goal(0.0, 0.0, False, 0)
            self.get_logger().info(f"IDLE game_progress={gp} (publish invalid goal)")
            return

        x, y, valid, goal_type, state = self.decide_goal(self.msg_latest)

        if state != self.last_state:
            self.get_logger().info(f"STATE CHANGE: {self.last_state} -> {state}")
            self.last_state = state

        publish_needed = True
        if valid and self.last_goal is not None:
            dx = x - self.last_goal[0]
            dy = y - self.last_goal[1]
            if (dx * dx + dy * dy) ** 0.5 < self.goal_eps:
                publish_needed = False

        if not valid:
            self.publish_goal(0.0, 0.0, False, 0)
            self.last_goal = None
            return

        if publish_needed:
            self.publish_goal(x, y, True, goal_type)
            self.last_goal = (x, y)


def main():
    rclpy.init()
    node = SentryDecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()