from wpiutil.log import DataLogReader, DataLogRecord, StartRecordData
from typing import Dict, List
import os
import struct

for file in os.listdir("logs"):
    reader = DataLogReader(f"logs/{file}")

    startRecordsByTopic: Dict[int, StartRecordData] = {}

    messagesByTopic: Dict[int, List[DataLogRecord]] = {}

    schemasByTypename: Dict[str, str] = {}

    for entry in reader:
        if entry.isStart():
            data = entry.getStartData()

            # print(f"Start record on topic {data.entry} : {data.name}")
            
            startRecordsByTopic[data.entry] = data
            messagesByTopic[data.entry] = []

        elif entry.isFinish():
            data = entry.getFinishEntry()
            # print(f"Finish record on topic {data}")
            pass
        elif entry.isControl():
            # print(f"Control record on topic {data}")
            pass
        elif entry.isSetMetadata():
            # print(f"Set metadata record on topic {data}")
            pass
        else:
            # print(f"Normal message published on topic ID {entry.getEntry()}: len {entry.getSize()}")
            if entry.getEntry() in startRecordsByTopic.keys():
                messagesByTopic[entry.getEntry()].append(entry)

            startData = startRecordsByTopic[entry.getEntry()]
            
            if ".schema/struct:" in startData.name:
                # print(startData.name)
                # print(entry.getRaw().decode())

                # huge hack
                if "TagDetection" in startData.name:
                    schemasByTypename[startData.name[startData.name.index("struct:") + len("struct:"):]] = "<Ldddddddd"
                if "Twist3d" in startData.name:
                    schemasByTypename[startData.name[startData.name.index("struct:") + len("struct:"):]] = "<dddddd"

    print(schemasByTypename)
    for topic, messageList in messagesByTopic.items():
        if "gtsam" not in startRecordsByTopic[topic].name:
            continue

        topic_typestring = startRecordsByTopic[messageList[0].getEntry()].type
        schema = schemasByTypename[topic_typestring[len("struct:"):]]

        decoded = []
        for message in messageList:
            decoded.append((message.getTimestamp(), struct.unpack(schema, message.getRaw())))

        print(decoded)


    print("")
    print(f"Topics in {file}:")
    for (key, topic) in startRecordsByTopic.items():
        # filter
        if "gtsam" not in topic.name:
            continue

        print("  " + topic.name + f": {len(messagesByTopic[key])} messages")

    print("=========")
