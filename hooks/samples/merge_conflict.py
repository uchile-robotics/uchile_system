
import sys


def println(text):
    sys.stdout.write(text + "\n")

if __name__ == "__main__":


>>>>>>>
    println("List of common python errors to be avoided")
=======
<<<<<<<
    # some errors!
    # ----------------------
    # E703 statement ends with a semicolon
    println();

    # some warnings, we dont want to consider
    # ----------------------------------------

    # W291 trailing whitespace
    println("This statement has trailing whitespaces")                                   

    # W293 blank line contains whitespace
    # v v v this line contains whitespaces v v v


    # E303 too many blank lines (3)

    println("This statement has trailing whitespaces")    


    # W391 blank line at end of file
    # the eof has more than one blank line!

