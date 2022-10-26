bag_name = "mocap_lateral_10_09_2.bag";
bag_file = rosbag(bag_name);
num_topics = height(bag_file.AvailableTopics);
publisher_map = containers.Map;
messageStruct_map = containers.Map;
topic_list = [];

for i=1:num_topics
    curr_topic = bag_file.AvailableTopics.Properties.RowNames{i};
    topic_list = [topic_list, curr_topic];
    curr_topic_type = string(bag_file.AvailableTopics.MessageType(i));
    bSel = select(bag_file,'Topic',curr_topic);
    msgStructs = readMessages(bSel, 'DataFormat','struct');
    messageStruct_map(curr_topic) = msgStructs;
end