import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

        # ==========================================
        # 1. THE SOURCE: Intel RealSense Camera
        # ==========================================
        realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
        rs_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_dir, 'rs_launch.py')),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'false',
                'rgb_camera.profile': '640x480x30',
                'align_depth.enable': 'false'
            }.items()
        )

        # ==========================================
        # 2. THE RESIZER: VIC Hardware Acceleration
        # ==========================================
        """
        resize_node = ComposableNode(
            name='resize_node',
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ResizeNode',
            parameters=[{
                'input_width': 640,    
                'input_height': 480,   
                'output_width': 518,
                'output_height': 518,
                'encoding_desired': 'rgb8',
                'keep_aspect_ratio': False
            }],
            remappings=[
                ('image', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
                ('resize/image', '/image_resized'),
                ('resize/camera_info', '/camera_info_resized')
            ]
        ) 
        """


    



        # ==========================================
        # 3. THE ENCODER: GPU Normalization
        # ==========================================
        # OLD C++ NODE (COMMENTED OUT SAFELY)
        # encoder_node = ComposableNode(
        #     name='dnn_image_encoder',
        #     package='isaac_ros_dnn_image_encoder',
        #     plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
        #     ...
        # )
        
        # NEW PYTHON LAUNCH INCLUSION
        encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
        dnn_image_encoder_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
            ),
            launch_arguments={
                'input_image_width': '640',
                'input_image_height': '480',
                'network_image_width': '518',
                'network_image_height': '518',
                'image_mean': '[0.485, 0.456, 0.406]',
                'image_stddev': '[0.229, 0.224, 0.225]',
                'attach_to_shared_component_container': 'True',
                'component_container_name': 'nitros_perception_container',
                'dnn_image_encoder_namespace': 'depth_encoder',
                'image_input_topic': '/camera/color/image_raw',
                'camera_info_input_topic': '/camera/color/camera_info',
                'tensor_output_topic': '/tensor_pub',
            }.items(),
        )

        # 2. THE ENCODER (Between Resize and TRT)
        encoder_dir_pedestrian = get_package_share_directory('isaac_ros_dnn_image_encoder')
        dnn_image_encoder_launch_pedestrian = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(encoder_dir_pedestrian, 'launch', 'dnn_image_encoder.launch.py')]
            ),
            launch_arguments={
                'input_image_width': '640',
                'input_image_height': '480',
                'network_image_width': '600',
                'network_image_height': '400',
                'image_mean': '[0.485, 0.456, 0.406]',
                'image_stddev': '[0.229, 0.224, 0.225]',
                'attach_to_shared_component_container': 'True',
                'component_container_name': 'nitros_perception_container',
                'dnn_image_encoder_namespace': 'pedestrian_encoder',
                'image_input_topic': '/camera/color/image_raw',
                'camera_info_input_topic': '/camera/color/camera_info',
                'tensor_output_topic': '/tensor_pub_pedestrian',
            }.items(),
        )


        # ==========================================
        # 4. THE BRAIN: TensorRT Inference
        # ==========================================
        tensor_rt_node = ComposableNode(
            name='tensor_rt_node',
            package='isaac_ros_tensor_rt',
            plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
            parameters=[{
                'model_file_path': '/workspaces/isaac_ros-dev/models/depth_anything_v2_vits.onnx',
                'engine_file_path': '/workspaces/isaac_ros-dev/models/depth_dla.engine',
                'input_tensor_names': ['input_tensor'],
                'input_binding_names': ['input'],
                'output_tensor_names': ['output'],
                'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_binding_names': ['identity_output'],
                'input_qos_history': 'keep_last',
                'input_qos_depth': 1,
                'force_max_batch_size': False,
                'force_engine_update': False,
            }],
            remappings=[
                ('tensor_pub', '/tensor_pub'),
                ('tensor_sub', '/depth_tensor_output')
            ]
        )
        pedestrian_trt_node = ComposableNode(
            name='pedestrian_trt_node',
            package='isaac_ros_tensor_rt',
            plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
            parameters=[{
                'engine_file_path': '/workspaces/isaac_ros-dev/models/checkpoint0033-lite.trt',
                'input_tensor_names': ['input_tensor'],   # Replace with your model's input name
                'input_binding_names': ['inp0'], 
                'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
                'output_tensor_names': ['out0','out1'], # Replace with your model's output name
                'output_binding_names': ['out0','out1'],
                'input_qos_history': 'keep_last',
                'input_qos_depth': 1,
                'force_max_batch_size': False,
                'force_engine_update': False,
            }],
            remappings=[
                ('tensor_sub', '/pedestrian_tensor_output'), # The 400x600 image we just made
                ('tensor_pub', '/tensor_pub_pedestrian') # The raw predictions
            ]
        )

        # ==========================================
        # 5. THE VISUALIZER: Custom C++ NITROS Node
        # ==========================================
        decoder_node = ComposableNode(
            name='depth_decoder_node',
            package='custom_rover_nodes',
            plugin='custom_rover_nodes::DepthDecoderNode',
            remappings=[
                ('/depth_tensor_output', '/depth_tensor_output'),
                ('/camera/depth/image_decoded', '/camera/depth/image_decoded')
            ]
        )
        pedestrian_follower_node = ComposableNode(
            name='PedestrianFollower',
            package='custom_rover_nodes',
            plugin='custom_rover_nodes::PedestrianFollower',
            
        )

        # ==========================================
        # 6. THE ZERO-COPY CAGE: The Container
        # ==========================================
        nitros_container = ComposableNodeContainer(
            name='nitros_perception_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
               #resize_node,
                tensor_rt_node,
                #decoder_node,
                pedestrian_trt_node,
                pedestrian_follower_node

            ],
            output='screen'
        )

        # This sits OUTSIDE the container, but INSIDE the launch file
        rover_control = Node(
        package='custom_rover_nodes',
        executable='rover_control_node.py',
        name='rover_control_node',
        output='screen'
        )
        rover_pedestrian_control = Node(
        package='custom_rover_nodes',
        executable='rover_pedestrian_control.py',
        name='rover_pedestrian_control',
        output='screen'
        )
        pedestrian_decoder = Node(
        package='custom_rover_nodes',
        executable='pedestrian_decoder_node.py',
        name='pedestrian_decoder_node',
        output='screen'
        )

        # Launch everything
        return LaunchDescription([
            rs_launch, 
            nitros_container, 
            dnn_image_encoder_launch,
            dnn_image_encoder_launch_pedestrian,
            #rover_control,
            rover_pedestrian_control,
            pedestrian_decoder
        ])
