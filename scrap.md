verify my undreestanding

make a node class

init function
make a publisher called system metrics
the publisher will put numbers into a 10 element queue of 32 bit floats
where does it get these numbers from?

make a timer
it runs 1 cycle per second
after each second, it will run the timer callback function

the class has a variable called last_total, which is the current timestamp of the cpu
you get the current timestamp using another helper method
also initialize last_idle with the current timestamp

the get_cpu_times() helper method gets the time from a linux system file called proc/stat
what is list(map(float why do you split and why 1:5? give an example))
why do you return the sum and what is times[3]

make another helper function called get_mem_usage
this reads another linux system file called proc/meminfo
readlines() counts the number of lines? or does it read everything in the meminfo file into the variable lnes?
the total memory available is line 1 of the meminfo file
the total memory that is free is line 2 of the meminfo file
this helper function returns the percentage of memory that is used

verify my understanding of
the timer callback function
get the total and idle time from the stat system file
why would you want to check if the total is greater than the last? how is it possible for the total to change?
update the last total and idle
get the memory usage from the /proc/meminfo/ system file
publish the cpu usage and memory usage
whats a float32multiarray()?
when you publish the message, what is it publishing to? is it just yelling into the void?