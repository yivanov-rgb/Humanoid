python3 - <<'PY'
import time
import copy
import rospy

from ainex_example.color_common import Common
from ainex_interfaces.msg import ObjectsInfo, ColorDetect


class RedCubeTransport(Common):

    block_roi = [5/8, 7/8, 3/8, 5/8]
    image_process_size = [160, 120]

    X_MIN = 250
    X_MAX = 270

    Y_MIN = 400
    Y_MAX = 420

    def __init__(self):
        rospy.init_node('red_cube_transport', anonymous=True)

        super().__init__('red_cube_transport', 500, 330)

        self.motion_manager.run_action('stand')

        self.object = None
        self.stage = 'x'
        self.busy = False
        self.finished = False

        rospy.Subscriber(
            '/object/pixel_coords',
            ObjectsInfo,
            self.object_callback
        )

        self.enter_func(None)
        self.start_srv_callback(None)

        params = self.set_blocks_color(
            ['red', 'green', 'blue'],
            self.block_roi,
            self.image_process_size
        )

        self.detect_pub.publish(params)

        rospy.loginfo('Red cube transport started')

        self.control_loop()

    def set_blocks_color(self, colors, roi, image_process_size):
        red = ColorDetect()

        red.color_name = colors[0]
        red.use_name = True
        red.detect_type = 'circle'

        red.roi.y_min = int(roi[0] * image_process_size[1])
        red.roi.y_max = int(roi[1] * image_process_size[1])
        red.roi.x_min = int(roi[2] * image_process_size[0])
        red.roi.x_max = int(roi[3] * image_process_size[0])

        red.image_process_size = image_process_size
        red.min_area = 10
        red.max_area = image_process_size[0] * image_process_size[1]

        green = copy.deepcopy(red)
        green.color_name = colors[1]

        blue = copy.deepcopy(red)
        blue.color_name = colors[2]

        return [red, green, blue]

    def object_callback(self, msg):
        red_cube = None

        for obj in msg.data:
            if obj.label == 'red' and obj.type == 'circle':
                red_cube = obj
                break

        self.object = red_cube

    def control_loop(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and not self.finished:

            if self.busy:
                rate.sleep()
                continue

            if self.object is None:
                self.busy = True

                print('RED CUBE LOST -> BACK ONE STEP')

                self.motion_manager.run_action('back_step')
                time.sleep(0.5)

                self.busy = False
                rate.sleep()
                continue

            x = self.object.x
            y = self.object.y
            radius = self.object.radius

            print(
                'RED:',
                'x=', x,
                'y=', y,
                'radius=', radius,
                'stage=', self.stage
            )

            self.object = None

            if self.stage == 'x':

                if x > self.X_MAX:
                    self.busy = True

                    print('CUBE LEFT -> MOVE RIGHT')

                    self.motion_manager.run_action('move_right')
                    time.sleep(0.1)

                    self.busy = False

                elif x < self.X_MIN:
                    self.busy = True

                    print('CUBE RIGHT -> MOVE LEFT')

                    self.motion_manager.run_action('move_left')
                    time.sleep(0.1)

                    self.busy = False

                else:
                    print('X ALIGNED -> START Y APPROACH')
                    self.stage = 'y'

            elif self.stage == 'y':

                if y < self.Y_MIN:
                    self.busy = True

                    print('CUBE TOO FAR -> FORWARD ONE STEP')

                    self.motion_manager.run_action(
                        'forward_one_step'
                    )
                    time.sleep(0.5)

                    self.busy = False

                elif y > self.Y_MAX:
                    self.busy = True

                    print('CUBE TOO CLOSE -> BACK ONE STEP')

                    self.motion_manager.run_action(
                        'back_step'
                    )
                    time.sleep(0.1)

                    self.busy = False

                else:
                    self.busy = True

                    print('TARGET POSITION REACHED')
                    print('FORWARD ONE STEP x2')
                    print('CLAMP LEFT')

                    # Подвод к кубу — утверждённая рабочая последовательность
                    self.motion_manager.run_action(
                        'forward_one_step'
                    )

                    self.motion_manager.run_action(
                        'forward_one_step'
                    )
                    time.sleep(0.1)

                    # Захват красного куба
                    self.motion_manager.run_action(
                        'clamp_left'
                    )
                    time.sleep(0.1)

                    print('CUBE GRAB SEQUENCE FINISHED')

                    # Поворот направо на 90°
                    print('TURN RIGHT 90 DEGREES')

                    for i in range(8):
                        self.motion_manager.run_action(
                            'turn_right'
                        )
                        time.sleep(0.1)

                    # Движение вперёд 12 шагов
                    print('FORWARD 12 STEPS')

                    for i in range(12):
                        self.motion_manager.run_action(
                            'forward'
                        )

                    # Положить куб
                    print('PLACE BLOCK')

                    self.motion_manager.run_action(
                        'place_block'
                    )

                    print('RED CUBE TRANSPORT FINISHED')

                    self.finished = True
                    rospy.signal_shutdown('red cube placed')

            rate.sleep()


RedCubeTransport()
rospy.spin()
PY