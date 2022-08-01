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

	https://github.com/kaito-IR/NakanoshimaChallengeDetectTask.git
導入したらcatkin_makeを実行してください．  
catkin_makeが終わったら

	source "catkinワークスペースのパス"/devel/setup.bash
を実行してください．これで導入は完了です．
# 実行方法
導入が済んだら，

	roslaunch yoros StartPersonDetect.launch 
を実行することで，YOROSノードを起動することができます．この際darknet_rosノードとrealsense2_cameraノードの起動も行われます．  
起動に成功すると，Realsenseのカメラ映像とマスク処理が施された2値化映像が別々のウィンドウで表示されます．  
マスク処理は，一般の通行人と，中之島チャレンジのスタッフ(オレンジのベストと青のキャップを着用)を見分けるために行なっています．  
色認識によってスタッフが着用しているベストとキャップの色を識別し，認識された領域の面積が300ピクセル以上の場合にその領域の重心座標を取得し，その座標がdarknet_rosによって識別されたperson(人間)のバウンティングボックスの中に入っていれば，その人間がオレンジのベストか青のキャップのどちらかを身につけていると判断し，RealsenseのDepthセンサによってその人との距離をmm単位で取得し，その値をStaffDistトピックに流します．  
逆に，オレンジのベストと青のキャップのどちらも身につけていないと判断された人間は，一般の通行人と判断され，その場合Depthセンサの値をPersonDistトピックに流します．
# HSVClick_ROSファイルについて
yorosパッケージ内のscriptsフォルダにあるHSVClick_ROS.pyファイルは，スタッフ検知のための色認識処理に使うHSVの値を調整するためのプログラムです．  
## 実行方法
まず，realsense2_cameraノードを起動し，Realsenseを起動します．

	roslaunch realsense2_camera rs_camera.launch
Realsense起動後，HSVClick_ROS.pyを実行します

	rosrun yoros HSVClick_ROS.py 
HSVCLick_ROSの実行に成功すると，Realsenseのカメラ映像と，マスク処理が施された2値化画像が別々のウィンドウで表示されます．  
この状態で，Realsenseのカメラ映像内のHSVの値を取得したい領域をマウスカーソルでクリックすると，その領域のHSVの値がコンソールに表示されます．  
さらに，2値化処理のupper変数とlower変数がクリックした領域のHSVの値に合わせて変化するため，本当にその値で正しく2値化出来ているのか確認することができます．  
あとはyorosパッケージ内のscriptsフォルダ内にあるSubscribeDetect.pyファイルの42~45行目付近の色認識の閾値変数(orange_min，orange_max，blue_min.blue_max)の値をこのプログラムで取得したHSVの値に手動で書き換えてください．
## 注意点
当然ですが，HSVClick_ROS.pyは対象のHSVの値を**大まかに**絞るためのプログラムです．このプログラムで取得できたHSVの値がそのまま利用できるとは限りません．  
そのため，SubscribeDetect.pyの閾値変数の値を書き換える場合は，ある程度の余裕を持たせてください．細かな値の調整は自力でお願いします．
# 後書き
このROSパッケージは本来中之島チャレンジでの通行人の自動検知及びスタッフの認識タスクの達成のために作ったパッケージですが，SubscribeDetect.pyの中身を書き換えれば様々な応用を利かせることができると思います．
