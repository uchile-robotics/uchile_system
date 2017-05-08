#!/usr/bin/python

import os
import sys


def _env(varname):
    if varname in os.environ:
        return os.environ[varname]
    print(" - Env variable is not set: '" + varname + "'")
    return ""


def _env_or_exit(varname):
    res = _env(varname)
    if not res:
        sys.exit(1)
    return res


def check_var_isset(varname):
    if not _env(varname):
        return False
    return True


def admin_goodbye():

    text = "Have some feedback?, please contact us at <"
    text += _env("ROBOT_EMAIL_DEVELOP")
    text += ">, or just contact "
    text += _env("ROBOT_SYSTEM_ADMIN") + ".\n"
    text += "\n"
    text += "kindly,\n"
    text += " - bender system admin - \n"
    text += "\n"
    print(text)
    return True


def shell_is_bash():
    if _env("CATKIN_SHELL") == "bash":
        return True
    return False


def shell_is_zsh():
    if _env("CATKIN_SHELL") == "zsh":
        return True
    return False


def shell_is_bash_or_zsh():
    return shell_is_bash() or shell_is_zsh()


def check_file_ext(filename, required_ext):
    remaining, ext = os.path.splitext(filename)
    if required_ext == ext:
        return True
    text = "Invalid file extension " + ext
    text += " for file: " + filename
    text += ". Required: " + required_ext
    print(text)
    return False


def get_filepath(filename):
    """
    Prints the file dirname. Relative paths are not converted into abspaths.

    >>> get_filepath("/usr/bin/foo")
    /usr/bin

    >>> get_filepath("../bin/foo path")
    ../bin

    >>> get_filepath("foo")
    .
    """
    dirname = os.path.dirname(filename)
    if not dirname:
        dirname = "."
    print(dirname)


if __name__ == '__main__':

    argc = len(sys.argv)
    if argc < 2:
        sys.exit(1)

    func = sys.argv[1]

    if func == "test":
        import doctest
        doctest.testmod()
        sys.exit(0)

    if func not in locals():
        print(" - Unknown function: " + func)
        sys.exit(1)

    method = locals()[func]
    args = sys.argv[2:argc]
    if method(*args) is False:
        sys.exit(1)
    sys.exit(0)
