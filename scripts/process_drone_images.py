from pyodm import Node
import glob

node = Node("localhost", 3000)

print(node.info())

images = glob.glob(
    "images/*.JPG"
)  ## finds all files ending in JPG inside images folder and stores their paths in a list

print("Found", len(images), "images")
print(images)

task = node.create_task(images, {"dsm": True})

print("task created!")
print("Task ID:", task.uuid)

task.wait_for_completion()

print("ODM finished!")

task.download_assets("results")
