import requests
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import logging
import sys
from datetime import datetime
import matplotlib.pyplot as plt

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Configuration
url = "http://localhost:8086"  # Change to correct adress
bucket = "bucket-name"  # Change to correct bucket-name
org = "org-name"  # Change to correct org-name
token = "your-api-token"  # Change to api-token with correct privileges

def ping_server(url):
    """Ping the InfluxDB server to check if it's reachable."""
    try:
        response = requests.get(f"{url}/ping")
        if response.status_code == 204:
            logger.info("Successfully connected to InfluxDB server.")
        else:
            logger.error("Failed to connect to InfluxDB server. Status code: %d", response.status_code)
            sys.exit(1)
    except requests.ConnectionError as e:
        logger.error("Error connecting to InfluxDB server: %s", e)
        sys.exit(1)

def create_client(url, token):
    """Create an InfluxDB client."""
    try:
        client = InfluxDBClient(url=url, token=token)
        logger.info("InfluxDB client created successfully.")
        return client
    except Exception as e:
        logger.error("Failed to create InfluxDB client: %s", e)
        sys.exit(1)

def write_data(client, org, bucket):
    """Write sample data to InfluxDB."""
    try:
        write_api = client.write_api(write_options=SYNCHRONOUS)
        point = Point("measurement_name").tag("tag_key", "tag_value").field("field_key", 42.0).time(datetime.utcnow(), WritePrecision.NS)
        write_api.write(bucket=bucket, org=org, record=point)
        logger.info("Data written successfully.")
    except Exception as e:
        logger.info("Failed to write data to InfluxDB: %s", e)


def query_data(client, org, bucket):
    """Query data from InfluxDB."""
    tables = None
    try:
        query = f'from(bucket: "{bucket}") |> range(start: -30d)'
        tables = client.query_api().query(query, org=org)
        for table in tables:
            count = 0
            for record in table.records:
                if count > 10: break
                else: count += 1
                logger.info(f'Time: {record.get_time()}, Value: {record.get_value()}')
    except Exception as e:
        logger.error("Failed to query data from InfluxDB: %s", e)
        sys.exit(1)

    return tables


def visualize_tables(tables):
    for table in tables:
        x_axis = []
        y_axis = []
        for record in table.records:
            x_axis.append(record.get_time())
            y_axis.append(record.get_value())

        plt.plot(x_axis, y_axis)
        plt.show()

def main():
    # Ping the server to check connectivity
    ping_server(url)

    # Create an InfluxDB client
    client = create_client(url, token)

    # Write sample data to InfluxDB
    write_data(client, org, bucket)

    # Query data from InfluxDB
    tables = query_data(client, org, bucket)

    # Visualize data from InfluxDB
    visualize_tables(tables)


if __name__ == "__main__":
    main()
