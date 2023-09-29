import csv

def parse_csv(file_contents):
    try:
        decoded_file = file_contents.decode('utf-8').splitlines()
        csv_reader = csv.reader(decoded_file)
        data = [row for row in csv_reader]
        return data
    except Exception as e:
        return None
