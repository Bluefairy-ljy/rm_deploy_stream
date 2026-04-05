运行代码：rosrun rm_deploy_stream rm_deploy_stream_main _enable_display:=false
（在nuc上要把图像显示关掉）
如果图传延时过长，考虑修改提高serial_sender_node.cpp里的第56行节点队列video_packet_sub_ = nh_.subscribe("/video_stream", 10, &SerialSenderNode::VideoPacketCB, this);
