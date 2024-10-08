from wpiutil.log import DataLogReader, DataLogRecord, StartRecordData
from typing import Dict, List
import os
import struct

for file in os.listdir("logs"):
    reader = DataLogReader(f"logs/{file}")

    startRecordMap: Dict[int, StartRecordData] = {}

    messagesByTopic: Dict[int, List[DataLogRecord]] = {}

    schemasByTopic: Dict[int, str] = {}

    for entry in reader:
        if entry.isStart():
            data = entry.getStartData()

            # print(f"Start record on topic {data.entry} : {data.name}")
            
            startRecordMap[data.entry] = data
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
            if entry.getEntry() in startRecordMap.keys():
                messagesByTopic[entry.getEntry()].append(entry)

            startData = startRecordMap[entry.getEntry()]
            
            if ".schema/struct:" in startData.name:
                print(startData.name)
                print(entry.getRaw().decode())

                # huge hack
                if "TagDetection" in startData.name:
                    schemasByTopic[entry.getEntry()] = "<Ldddddddd"
                if "Twist3d" in startData.name:
                    schemasByTopic[entry.getEntry()] = "<dddddd"

    

    # print("")
    # print(f"Topics in {file}:")
    # for (key, topic) in startRecordMap.items():
    #     print("  " + topic.name)

    print("=========")
