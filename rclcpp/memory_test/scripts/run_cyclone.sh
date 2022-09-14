echo 
echo "CycloneDDS"
echo 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

./default_nodes > cyclone_default_nodes.txt
./nodes_params_off > cyclone_nodes_params_off.txt
./default_subs_params_off > cyclone_default_subs_params_off.txt
./default_pubs_params_off > cyclone_default_pubs_params_off.txt
./default_clients_params_off > cyclone_default_clients_params_off.txt
./default_services_params_off > cyclone_default_services_params_off.txt
./nodes_params_off_logging_on > cyclone_nodes_params_off_logging_on.txt
./pub_sub_diff_topic > cyclone_pub_sub_diff_topic.txt
./pub_sub_same_topic > cyclone_pub_sub_same_topic.txt
./pub_sub_diff_msg_type > cyclone_pub_sub_diff_msg_type.txt
./cli_serv_diff_topics > cyclone_cli_serv_diff_topics.txt
./cli_serv_same_topics > cyclone_cli_serv_same_topics.txt
./pub_sub_big_history_size > cyclone_pub_sub_big_history_size.txt
./pub_sub_big_message_size > cyclone_pub_sub_big_message_size.txt
