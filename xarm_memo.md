# 設計メモ

1. 基本的な流れ
   - eval_real.py -> bimanual_umi_env.py -> 
     - https://github.com/robograffitti/universal_manipulation_interface/blob/develop/eval_real.py#L175-L197
       - 実機を扱う env 変数を BimanualUmiEnv でインスタンス化
     - https://github.com/robograffitti/universal_manipulation_interface/blob/develop/umi/real_world/bimanual_umi_env.py#L233-L242
       - franka を扱うクラスをインスタンス化
     - https://github.com/robograffitti/universal_manipulation_interface/blob/develop/scripts_real/launch_franka_interface_server.py
       - 実機との通信を行うインタフェースサーバー
       - Polymetis と zerorpc いうものを ROS の代わりに使っている（ROSで実装する場合はこれはおそらく不要）
     - https://github.com/robograffitti/universal_manipulation_interface/blob/develop/umi/real_world/franka_interpolation_controller.py
       - 上記の polymetis と zerorpc を動かしているサーバー経由で umi が生成した司令を送るためのコントローラ
       - xArm を moveit で動かす場合、polymetis と zerorpc は使わず、これを ROS の python ノードとして実装する

# franka と UR の違い

1. FrankaInterface がある
- zerorpc 等、frankaの通信インタフェース固有の実装
2. 初期化パラメータ
- ring_buffer に格納する shm_manager, example, frequency 以外は違う
3. receive_keys
- franka は ActualTCPPose, ActualQ, ActualQd だけでよい
- 手先姿勢、関節角、関節速度をmoveitからとれればよい
4. init と run 以外は同じ
5. FrankaInterfaceの初期化
6. 初期姿勢の送信
7. while keep_running:
- 手先位置の送信コマンド
- tx_flange の行列変換
- state 更新
8. finally
- ロボットに合わせた終了処理

# 設計メモ（古い情報）
1. bimanual_umi_env.py  
   - 以下で robot_type を xarm に設定する分岐を追加
     - https://github.com/robograffitti/universal_manipulation_interface/blob/develop/umi/real_world/bimanual_umi_env.py#L233-L242
   - FrankaInterpolationController の下に XArmInterpolationController を追加
   - 必要なパラメータの読み込み
   - 必要なメソッドの実装を franka を参考に実装
   - eval_real.py の中で BimanualUmiEnv としてインスタンス化
     - aaa
     - bbb
2. xarm_interpolation_controller.py を新規作成  
   - umi -> controller -> server -> moveit
   - serverを省略してここでmoveitと直接通信してもいいかも
   - while keep_runningにrosのループを書く？
3. launch_xarm_interface_server.py を新規作成
   - xarm の moveit と通信するインタフェース
   - moveit が ここに相当する設計なら不要かも