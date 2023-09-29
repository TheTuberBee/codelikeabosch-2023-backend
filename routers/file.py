from fastapi import APIRouter, UploadFile, HTTPException

from common.csv import parse_csv


router = APIRouter()


@router.post("/file")
async def file_upload(file: UploadFile):
    if file.content_type != "text/csv":
        raise HTTPException(400, "File must be in CSV format.")

    file_contents = await file.read()
    parsed_data = parse_csv(file_contents)
    
    if parsed_data:
        return { "data": parsed_data }
    else:
        raise HTTPException(400, "Unable to parse file.")
