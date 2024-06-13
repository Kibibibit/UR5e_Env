

class ANSIColour:
    RED="31"
    GREEN="32"
    YELLOW="33"
    BLUE="34"

def colour_string(colour:str, string:str)->str:
    return "\033["+colour+"m"+string+"\033[0m"