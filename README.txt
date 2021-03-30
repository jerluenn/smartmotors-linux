Testing for Class 5 Linux Driver functions
SUSE Linux 2.6.37.1

gcc v 4.5.1



1) Boot Linux machine


2) Open Terminal and Navigate to Linux_test folder
	
	check to see if machine has detected the flash drive by getting to it through the file manager

	Folder must be moved off of removeable disk

	Nav to media drive: 	    			cd /media
	List to find flash drive:   			ls  
	Nav to correct flash drive:	    		cd A2D5-57A4
	Move Linux_test folder off of drive:   		mv Linux_test /home/profitest/HQ/


3) Navigate to the desired path
	
	cd /home/profitest/HQ/Linux_test

4) Choose test in main.c
	
	vim main.c
	press i once in vim to be able to make edits
	press esc when done
	press :x to save changes and exit vim

5) Compile and make executable
	
	Command: make -f makefile

6) Run executable 
	
	make sure Motor is on and connected

	Command:  sudo ./test
	password: password

	