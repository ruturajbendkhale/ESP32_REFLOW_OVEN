Import("env")

def prepare_www(source, target, env):
    print("Preparing web files...")
    # The web files are already in the data directory
    # PlatformIO will handle the upload automatically

env.AddPreAction("buildfs", prepare_www)
env.AddPreAction("uploadfs", prepare_www) 