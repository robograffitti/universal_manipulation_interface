# 設計メモ

1. bimanual_umi_env.py
   FrankaInterpolationController の下に XArmInterpolationController を追加
   必要なパラメータの読み込み
   必要なメソッドの実装を franka を参考に実装
   eval_real.py の中で BimanualUmiEnv としてインスタンス化
2. launch_xarm_interface_server.py を新規作成
   xarm の moveit と通信するインタフェース
   moveit が ここに相当する設計なら不要かも
3. xarm_interpolation_controller.py を新規作成
   umi -> controller -> server -> moveit
   serverを省略してここでmoveitと直接通信してもいいかも
   while keep_runningにrosのループを書く？