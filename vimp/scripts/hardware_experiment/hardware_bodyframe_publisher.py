from fake_bodyframe_publisher import BodyFramePosePublisher


if __name__ == '__main__':
    pose_publisher = BodyFramePosePublisher('/B1_pose', wait_key=True, listen_to_ack=False)
    pose_publisher.run()