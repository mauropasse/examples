echo 
echo "StubDDS"
echo 
export RMW_IMPLEMENTATION=rmw_stub_cpp

./default_nodes > stub_default_nodes.txt
./nodes_params_off > stub_nodes_params_off.txt
./default_subs_params_off > stub_default_subs_params_off.txt
./default_pubs_params_off > stub_default_pubs_params_off.txt
./default_clients_params_off > stub_default_clients_params_off.txt
./default_services_params_off > stub_default_services_params_off.txt
./nodes_params_off_logging_on > stub_nodes_params_off_logging_on.txt
./pub_sub_diff_topic > stub_pub_sub_diff_topic.txt
./pub_sub_same_topic > stub_pub_sub_same_topic.txt
./pub_sub_diff_msg_type > stub_pub_sub_diff_msg_type.txt
./cli_serv_diff_topics > stub_cli_serv_diff_topics.txt
./cli_serv_same_topics > stub_cli_serv_same_topics.txt
./pub_sub_big_history_size > stub_pub_sub_big_history_size.txt
./pub_sub_big_message_size > stub_pub_sub_big_message_size.txt
