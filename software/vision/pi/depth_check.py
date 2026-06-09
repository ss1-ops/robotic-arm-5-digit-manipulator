#!/usr/bin/env python3
"""Quick check that the depth node is producing metric target points. Collects
/stereo/target/{state,distance} for ~5s and writes a summary to ~/depth_check.out."""
import os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class C(Node):
    def __init__(self):
        super().__init__("depth_check")
        self.states = {}
        self.dists = []
        self.create_subscription(String, "/stereo/target/state", self.scb, 10)
        self.create_subscription(Float32, "/stereo/target/distance", self.dcb, 10)
    def scb(self, m): self.states[m.data] = self.states.get(m.data, 0) + 1
    def dcb(self, m):
        if m.data > 0: self.dists.append(m.data)

def main():
    rclpy.init(); n = C(); t = time.time()
    while rclpy.ok() and time.time() - t < 5:
        rclpy.spin_once(n, timeout_sec=0.3)
    out = [f"states={n.states}"]
    if n.dists:
        import statistics as st
        out.append(f"valid_dist samples={len(n.dists)} "
                   f"min={min(n.dists)*100:.0f}cm median={st.median(n.dists)*100:.0f}cm "
                   f"max={max(n.dists)*100:.0f}cm")
    else:
        out.append("no valid distances (state never TRACK, or nothing in gate)")
    open(os.path.expanduser("~/depth_check.out"), "w").write("\n".join(out) + "\n")
    n.destroy_node()

if __name__ == "__main__":
    main()
