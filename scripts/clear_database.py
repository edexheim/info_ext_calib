
import os
import sys

try:
  dataset_dir = sys.argv[1]
except IndexError:
  print("Usage: " + os.path.basename(__file__) + " <database subfolder>")
  sys.exit(1)


folder = '../database/' + dataset_dir
for filename in os.listdir(folder):
  if filename == '.gitignore':
    continue
  file_path = os.path.join(folder, filename)
  try:
    if os.path.isfile(file_path):
        os.remove(file_path)
  except Exception as e:
    print('Failed to delete %s. Reason: %s' % (file_path, e))