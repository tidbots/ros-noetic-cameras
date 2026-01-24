#!/usr/bin/env python3
import time
import rospy
from topic_tools.srv import MuxSelect

def call_select(service_name: str, topic: str) -> None:
    rospy.loginfo(f"[mux_select] wait {service_name}")
    rospy.wait_for_service(service_name, timeout=30.0)
    srv = rospy.ServiceProxy(service_name, MuxSelect)
    srv(topic)
    rospy.loginfo(f"[mux_select] {service_name} -> {topic}")

def main():
    rospy.init_node("mux_select", anonymous=True)

    src = rospy.get_param("~default_source", "xtion").strip()

    webcam_rgb  = rospy.get_param("~webcam_rgb")
    webcam_info = rospy.get_param("~webcam_info")

    xtion_rgb        = rospy.get_param("~xtion_rgb")
    xtion_info       = rospy.get_param("~xtion_info")
    xtion_depth      = rospy.get_param("~xtion_depth")
    xtion_depth_info = rospy.get_param("~xtion_depth_info")
    xtion_points     = rospy.get_param("~xtion_points")

    rs_rgb        = rospy.get_param("~rs_rgb")
    rs_info       = rospy.get_param("~rs_info")
    rs_depth      = rospy.get_param("~rs_depth")
    rs_depth_info = rospy.get_param("~rs_depth_info")
    rs_points     = rospy.get_param("~rs_points")

    # mux node service出現待ち
    time.sleep(1.0)

    if src == "webcam":
        rgb, info = webcam_rgb, webcam_info
        # webcamはdepth無し → xtionへフォールバック
        depth, depth_info, points = xtion_depth, xtion_depth_info, xtion_points
    elif src == "realsense":
        rgb, info = rs_rgb, rs_info
        depth, depth_info, points = rs_depth, rs_depth_info, rs_points
    else:  # xtion default
        rgb, info = xtion_rgb, xtion_info
        depth, depth_info, points = xtion_depth, xtion_depth_info, xtion_points

    try:
        call_select("/mux_rgb_image/select", rgb)
        call_select("/mux_rgb_info/select", info)
        call_select("/mux_depth_image/select", depth)
        call_select("/mux_depth_info/select", depth_info)
        call_select("/mux_points/select", points)
    except Exception as e:
        rospy.logwarn(f"[mux_select] select failed: {e}")

    rospy.loginfo("[mux_select] ready.")
    rospy.spin()

if __name__ == "__main__":
    main()

