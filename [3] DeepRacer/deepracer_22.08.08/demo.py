from keyPoller import KeyPoller
import time

key = KeyPoller()

with key as poller:

    while True:
        char = poller.poll()

        if char != None:
            print("=" * 10)
            print("YOUR KEY IS ")
            print(char)

        if char == 's':
            while True:
                print("doing")
                time.sleep(0.5)

                ###############
                #key interrupt#
                ###############
                char = poller.poll()


                if char != None :
                    print("=" * 10)
                    print("YOUR KEY IS ")
                    print(char)
                if char == 'e':
                    print("show graph")
                    break
