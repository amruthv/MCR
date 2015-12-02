def runNTimes(m, n):
    times = []
    for i in range(n):
        a = time.time()
        p = multiprocessing.Process(target=m)
        p.start()
        p.join(120)
        # If thread is active
        if p.is_alive():
            print "m is running... let's kill it..."
            # Terminate m
            p.terminate()
            p.join()
            times.append(float('inf'))
        else:
            b = time.time()
            times.append(b-a);
    return times

def prof(test, n=50):
    import cProfile
    import pstats
    try:
        cProfile.run(test, 'prof')
    except:
        print 'Done'
    p = pstats.Stats('prof')
    p.sort_stats('cumulative').print_stats(n)