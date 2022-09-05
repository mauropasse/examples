echo 
echo "Fastrtps"
echo 
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

./default_nodes > fast_default_nodes.txt
./nodes_params_off > fast_nodes_params_off.txt
./default_subs_params_off > fast_default_subs_params_off.txt
./default_pubs_params_off > fast_default_pubs_params_off.txt
./default_clients_params_off > fast_default_clients_params_off.txt
./default_services_params_off > fast_default_services_params_off.txt
./nodes_params_off_logging_on > fast_nodes_params_off_logging_on.txt
./pub_sub_diff_topic > fast_pub_sub_diff_topic.txt
./pub_sub_same_topic > fast_pub_sub_same_topic.txt
./pub_sub_diff_msg_type > fast_pub_sub_diff_msg_type.txt
./cli_serv_diff_topics > fast_cli_serv_diff_topics.txt
./cli_serv_same_topics > fast_cli_serv_same_topics.txt
./pub_sub_big_history_size > fast_pub_sub_big_history_size.txt
./pub_sub_big_message_size > fast_pub_sub_big_message_size.txt
