import threading
import time

def thread_1():
    print("thread_1 forks")
    for i in range(10):
        print("thread_1: ", i)
        time.sleep(0.5)
    print("thread_1 done")

def thread_2():
    print("thread_2 forks")
    for i in range(5):
        print("thread_2: ", i)
        time.sleep(0.5)
    print("thread_2 done")

t1 = threading.Thread(target=thread_1)
t2 = threading.Thread(target=thread_2)

# fork
print("main thread forks")
t1.start()
t2.start()
print("main thread done")

t1.join()
t2.join()

print("program successfully finished...")
