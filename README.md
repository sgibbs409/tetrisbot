# Tetris BOT!!!!

You know what this is; made for the PUMA arm with love

### the game

In the game directory there is a single python file that relies on [PyGame](http://pygame.org).  On Windows and Mac this can be installed from the website, and on ubuntu via "apt-get install python-pygame".

After pygame is installed, run via the command "python tetromino.py"

When run it allows you to play the game, and also outputs to the command line commands for the Puma, like in the [example log file](example_game_log.txt)

### SCL

First of all the tetrisbot directory has to reside in 'scl-manips-v2/tutorial/'.  If it is anywhere else it needs to be moved there first (git does not mind)

Then you can run them together as "sh make_rel.sh && python tetromino.py | ./tetrisbot"

### Running on the PUMA

1) Install pygame
2) Run the game: in simpleclient2014/tetris "python tetromino.py"
3) Compile and run the code: in simpleclient2014 "sh README.txt", selecting the default for both servers
