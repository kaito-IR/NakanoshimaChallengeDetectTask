## これはrealsense2_cameraパッケージとdarknet_rosパッケージを用いて，Realsenseで撮った映像上から人間を検知し，検知した人間との距離をROSTopic上に流すパッケージです．
# 実装環境  
端末：Jetson Xavier NX  
カメラ：Realsense D435i  
OS：Ubuntu18.04LTS  
ROS：Melodic　　
# 実装方法
### Step1.
catkinワークスペースのsrc内にdarknet_rosパッケージ及びrealsense2_cameraパッケージを導入してください.  
[darknet_rosパッケージ](https://github.com/leggedrobotics/darknet_ros)及び[realsense2_cameraパッケージ](https://github.com/IntelRealSense/realsense-ros)の導入方法はそれぞれのGitHubを参照してください.  
### Step2.
catkinワークスペースのsrc内にyorosパッケージを導入します．導入方法は

	git clone
