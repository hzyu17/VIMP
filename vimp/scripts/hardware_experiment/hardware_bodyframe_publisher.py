from fake_bodyframe_publisher import PosePublisher


if __name__ == '__main__':
    pose_publisher = PosePublisher('/B1_pose', wait_key=True, listen_to_ack=False)
    pose_publisher.run()