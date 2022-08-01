## これはrealsense2_cameraパッケージとdarknet_rosパッケージを用いて，Realsenseで撮った映像上から人間を検知し，検知した人間との距離をROSTopic上に流すパッケージです．
# 使用環境  
端末：Jetson Xavier NX  
カメラ：Realsense D435i  
OS：Ubuntu18.04LTS  
ROS：Melodic　　
# 導入方法
### Step1.
catkinワークスペースのsrc内にdarknet_rosパッケージ及びrealsense2_cameraパッケージを導入してください.  
[darknet_rosパッケージ](https://github.com/leggedrobotics/darknet_ros)及び[realsense2_cameraパッケージ](https://github.com/IntelRealSense/realsense-ros)の導入方法はそれぞれのGitHubを参照してください.  
### Step2.
catkinワークスペースのsrc内にgitコマンドを使ってyorosパッケージを導入します．

	git clone https://github.com/kaito-IR/NakanoshimaChallenge2022.git
導入したらcatkin_makeを実行を実行してください．  
catkin_makeが終わったら

	source "catkinワークスペースのパス"/devel/setup.bash
を実行してください．これで導入は完了です．
# 実行方法
導入が済んだら，

	roslaunch yoros StartPersonDetect.launch 
を実行することで,YOROSノードを起動することができます．この際，darknet_rosノードとrealsense2_cameraノードの起動も自動で行われます．  
起動に成功すると，Realsenseのカメラ映像とマスク処理が施された2値化映像が別々のウィンドウで表示されます．  
