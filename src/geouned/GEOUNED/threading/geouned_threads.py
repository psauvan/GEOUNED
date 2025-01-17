
import sys
import threading 
import time
from tqdm import tqdm

class ReturnValueThread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.result = None

    def run(self):
        if self._target is None:
            return  # could alternatively raise an exception, depends on the use case
        try:
            self.result = self._target(*self._args, **self._kwargs)
        except Exception as exc:
            print(f'{type(exc).__name__}: {exc}', file=sys.stderr)  # properly handle the exception

    def join(self, *args, **kwargs):
        super().join(*args, **kwargs)
        return self.result

def thread_release(started_threads,finished_threads):
    finished_threads.clear()
    while True:
        running_threads = threading.enumerate()
        my_thread_number = len(running_threads)-2
        if my_thread_number != len(started_threads):         
            for thread in started_threads:
                if thread not in running_threads:
                    finished_threads.append(thread)
            if len(finished_threads) != 0:
                for thread in finished_threads:
                    started_threads.remove(thread)
                break    
        time.sleep(0.05)



def ThreadPoolExecutor(target, argument_list, target_options=[], max_thread=4, progress_bar_label='Decompose',add_index=False):

    started_threads = []
 #   results = []

    for i,arg in enumerate(tqdm(argument_list, desc=progress_bar_label)):
        finished_threads = []
        if add_index:
            args = (arg,i, *target_options )
        else:    
            args = (arg, *target_options )
        #thread = ReturnValueThread(target=target, args=args) 
        thread = threading.Thread(target=target, args=args)

        thread.start()
        started_threads.append(thread)

        if len(started_threads)<max_thread:
            continue

        thread_release(started_threads,finished_threads)
#        results.extend(t.result for t in finished_threads)

#    else:
#        for thread in started_threads:
#            results.append(thread.join())    

#    return results            

