from fastapi import UploadFile, HTTPException, Depends
from pydantic import BaseModel

from typing import Annotated

from common.csv import parse_csv
from common.world import RawObject, Tick


async def upload_file(
    file: UploadFile
) -> list[list[str]]:

    if file.content_type != "text/csv" and file.content_type != "application/vnd.ms-excel":
        raise HTTPException(400, "File must be in CSV format.")

    file_contents = await file.read()
    parsed_data = parse_csv(file_contents)

    if parsed_data:
        return parsed_data
    else:
        raise HTTPException(400, "Unable to parse file.")


async def get_filename(
    file: UploadFile
) -> str:
    return file.filename


async def parse_dataset(
    raw_data: Annotated[list[list[str]], Depends(upload_file)],
) -> list[Tick]:
    # TODO: parse 2d str list into a list of ticks

    # create a dict with headers and indices
    headers = raw_data.pop(0)
    headers[0] = "_"
    headers_dict = {header: index for index, header in enumerate(headers)}

    distance_headers = [
        {"x": "FirstObjectDistance_X",  "y": "FirstObjectDistance_Y"},
        {"x": "SecondObjectDistance_X", "y": "SecondObjectDistance_Y"},
        {"x": "ThirdObjectDistance_X",  "y": "ThirdObjectDistance_Y"},
        {"x": "FourthObjectDistance_X", "y": "FourthObjectDistance_Y"},
    ]
    speed_headers = [
        {"x": "FirstObjectSpeed_X",  "y": "FirstObjectSpeed_Y"},
        {"x": "SecondObjectSpeed_X", "y": "SecondObjectSpeed_Y"},
        {"x": "ThirdObjectSpeed_X",  "y": "ThirdObjectSpeed_Y"},
        {"x": "FourthObjectSpeed_X", "y": "FourthObjectSpeed_Y"},
    ]

    # parse
    out_ticks: list[Tick] = []
    for row in raw_data:
        objects = []
        for i in range(4):
            x = float(row[headers_dict[distance_headers[i]["x"]]])
            y = float(row[headers_dict[distance_headers[i]["y"]]])
            vx = float(row[headers_dict[speed_headers[i]["x"]]])
            vy = float(row[headers_dict[speed_headers[i]["y"]]])
            if x != 0 and y != 0:
                objects.append(
                    RawObject(
                        x_rel = x / 128,    # m
                        y_rel = y / 128,    # m
                        vx_rel = vx / 256,  # m/s
                        vy_rel = vy / 256,  # m/s
                    )
                )

        out_ticks.append(Tick(
            index = int(row[0]),
            time = float(row[headers_dict["Timestamp"]]),
            host_yaw_rate = float(row[headers_dict["YawRate"]]),
            host_speed = float(row[headers_dict["VehicleSpeed"]]) / 256, # m/s
            objects = objects
        ))

    return out_ticks
