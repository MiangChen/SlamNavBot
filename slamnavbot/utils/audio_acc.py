import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class ASRSubscriber(Node):
    def __init__(self):
        super().__init__('asr_subscriber')
        # 创建订阅者，订阅话题 rt/audio_msg，消息类型为String
        self.subscription = self.create_subscription(
            String,
            'rt/audio_msg',
            self.listener_callback,
            10  # 队列长度
        )
        self.subscription  # 防止未使用变量警告
        self.get_logger().info('ASR订阅节点已启动，等待消息...')

    def listener_callback(self, msg):
        try:
            # 解析JSON字符串
            data = json.loads(msg.data)

            # 提取并打印关键信息（按需修改）
            self.get_logger().info(f"\n【ASR消息】\n"
                                  f"文本: {data.get('text', '')}\n"
                                  f"角度: {data.get('angle', 0)}°\n"
                                  f"置信度: {data.get('confidence', 0.0):.2f}\n"
                                  f"语言: {data.get('language', '')}\n"
                                  f"最终结果: {data.get('is_final', False)}")

            # 示例：根据内容执行动作
            if "你好" in data.get('text', ''):
                self.get_logger().info("检测到问候语，触发响应流程...")

        except json.JSONDecodeError:
            self.get_logger().error('JSON解析失败！原始数据: %s' % msg.data)

def main():
    rclpy.init()
    node = ASRSubscriber()
    try:
        rclpy.spin(node)  # 持续监听话题
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

