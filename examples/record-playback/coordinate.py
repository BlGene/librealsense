import subprocess
import time

def main():
    recording_proc = None


    while True:
        value = input('Enter input:')
        
        # start recording
        if value == "MACRO?3":
            if recording_proc is not None:
                continue
            
            print("Starting Recording")
            recording_proc = subprocess.Popen(["./rs-record-minimal"], 
                stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            
            subprocess.run(["cvlc","start.ogg","vlc://quit"]) 

        # stop recording
        elif value == "MACRO?A":
            if recording_proc is None:
                continue

            recording_proc.stdin.write(b"\n")
            for i in range(10):
                if recording_proc.poll():
                    time.sleep(1)
                else:
                    break
            recording_proc.terminate()
            recording_proc = None
            subprocess.run(["cvlc","stop.ogg","vlc://quit"]) 

        else:
            print("Unknow command: ", value)
            pass

if __name__ == "__main__":
    main()
