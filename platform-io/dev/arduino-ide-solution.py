
import os, shutil
import argparse
import configparser

def parseConfigFile(config_file):
  print ("\tParsing '" + config_file +"' file...")
  config = configparser.ConfigParser()
  config.read(config_file)
  libs_paths = config['env:esp32dev']['build_flags'].replace('-I./', '').split('\n')
  libs_infos = []
  for lib in libs_paths:
    lib_name = []
    for val in lib.split('/'):
      if (val != '' and val != "src" and val != "lib"):
        lib_name.append(val)
    if (len(lib_name) == 1):
      libs_infos.append([lib, lib_name[0]])
  print("\tdone\n")
  return libs_infos

def yes_no(answer):
    yes = set(['yes','y', 'ye', ''])
    no = set(['no','n'])
     
    while True:
        choice = raw_input(answer).lower()
        if choice in yes:
           return True
        elif choice in no:
           return False
        else:
           print "Please respond with 'yes' or 'no'"

description = """GNUVario ArduinoIDE Solution Sync
  This script can:
  * generates ArduinoIDE solution from the src repository
  * synchronize the solution to the src repository (support any addition or modification in the libs)
  NB1: Uses the 'platformio.ini' file, so keep it up-to-date !
  NB2: The deletion of a lib is NOT supported, you'll have to do it manually.
  """

# Initiate the parser
parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)

# Add long and short argument
parser.add_argument("--create", "-c", action='store_true', help="Create the ArduinoIDE solution")
parser.add_argument("--sync", "-s", action='store_true', help="Synchronize ArduinoIDE changes to PlatformIO sources")
parser.add_argument("--folder", "-f", default="../arduino-ide", help="Name of the ArduinoIDE solution folder (default: '../arduino-ide')")

# Read arguments from the command line
args = parser.parse_args()

if args.create:
  if (os.path.isdir(args.folder)):
    print ("ERROR: The solution folder '"+ args.folder + "' already exists.\nIf you want to override it, remove it first or choose another folder using -f option.\n")
    import sys ; sys.exit()

  print("Generating '"+ args.folder +"' solution folder\n")

  libs_infos = parseConfigFile("platformio.ini")

  print ("\tCopying Main cpp file...")
  source_path = os.path.join("src", "Gnuvario-E.cpp")
  target_path = os.path.join(args.folder, "Gnuvario-E", "Gnuvario-E.ino")
  os.makedirs(os.path.dirname(target_path))
  shutil.copy(source_path, target_path)
  print("\tdone\n")

  f = open(os.path.join(args.folder, ".gitignore"), "w")
  f.write("*\n")
  f.close()
  
  print ("\tCopying libs...")
  for lib_infos in libs_infos:
    lib_path, lib_name = lib_infos
    if (os.path.isdir(lib_path)):
      target_path = os.path.join(args.folder, "libraries", lib_name)
      print("\t\tCopying: '" + lib_path + "' lib into '" + target_path + "'")
      destination = shutil.copytree(lib_path, target_path)
    else:
      print("ERROR: '" + lib_path + "' lib does no exist")
      import sys ; sys.exit()
  print ("\tCopying libs...done")

elif args.sync:
  print("Synchronizing '"+ args.folder +"' with PlatformIO sources\n")

  libs_infos = parseConfigFile("platformio.ini")

  print ("\tSynchronizing Main cpp file...")
  target_path = os.path.join("src", "Gnuvario-E.cpp")
  source_path = os.path.join(args.folder, "Gnuvario-E", "Gnuvario-E.ino")
  if (os.path.exists(source_path)):
    shutil.copy(source_path, target_path)
  else:
    print("ERROR: '" + source_path + "' does no exist")
    import sys ; sys.exit()
  print("\tdone\n")

  for lib_infos in libs_infos:
    lib_path, lib_name = lib_infos
    arduino_lib_path = os.path.join(args.folder, "libraries", lib_name)
    if (os.path.isdir(arduino_lib_path)):
      if (os.path.isdir(lib_path)):
        print("\t\tSynchronizing: '" + arduino_lib_path + "' into '" + lib_path + "'")
        shutil.rmtree(lib_path)
      else:
        if (not yes_no(lib_path + " does not exist in the src. Do you really want to add this lib ? (Type 'yes' or 'no')")):
          continue
        print("\t\tCopying new lib: '" + arduino_lib_path + "' in '" + lib_path + "'")
      shutil.copytree(arduino_lib_path, lib_path)
    else:
      print("ERROR: '" + arduino_lib_path + "' lib does no exist")
      import sys ; sys.exit()