from wpiutil.log import DataLogReader, DataLogRecord, StartRecordData
from typing import Dict, List
import os
import struct

for file in os.listdir("logs"):
    reader = DataLogReader(f"logs/{file}")

    topics: Dict[int, StartRecordData] = {}

    messages: Dict[int, List[DataLogRecord]] = {}

    schemas: Dict[int, str] = {}

    for entry in reader:
        if entry.isStart():
            data = entry.getStartData()

            print(f"Start record on topic {data.entry} : {data.name}")
            
            topics[data.entry] = data
            messages[data.entry] = []

            if "struct:" in data.name:
                print("Typestring is: " + data.type)
                print("Metadata is: " + data.metadata)

        elif entry.isFinish():
            data = entry.getFinishEntry()
            print(f"Finish record on topic {data}")
            pass
        elif entry.isControl():
            print(f"Control record on topic {data}")
            pass
        elif entry.isSetMetadata():
            print(f"Set metadata record on topic {data}")
            pass
        else:
            print(f"Normal message published on topic ID {entry.getEntry()}: len {entry.getSize()}")
            if entry.getEntry() in topics.keys():
                messages[entry.getEntry()].append(entry)

            startData = topics[entry.getEntry()]
            if ".schema/struct:" in startData.name:
                print(entry.getRaw())

    print("")
    print(f"Topics in {file}:")
    for (key, topic) in topics.items():
        print("  " + topic.name)

    print("=========")
