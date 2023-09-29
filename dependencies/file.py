from fastapi import UploadFile, HTTPException, Depends
from pydantic import BaseModel

from typing import Annotated

from common.csv import parse_csv
from common.world import RawObject, Tick


async def upload_file(
    file: UploadFile
) -> list[list[str]]:
    
    if file.content_type != "text/csv":
        raise HTTPException(400, "File must be in CSV format.")

    file_contents = await file.read()
    parsed_data = parse_csv(file_contents)
    
    if parsed_data:
        return parsed_data
    else:
        raise HTTPException(400, "Unable to parse file.")


async def parse_dataset(
    raw_data: Annotated[list[list[str]], Depends(upload_file)],
) -> list[Tick]:
    # TODO: parse 2d str list into a list of ticks
    pass
