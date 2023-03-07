


## Orientation
currOrientation = 0
degreesToTurn = 0

##
# Change the orientation to where Robomow is looking from the matrix perspective
# after that it has turned to the left
##
def changeOrLeft(degree):
    #     if(currOrientatiion+degree <= 180):
    #         currOrientation = currOrientation + degree
    #     if(currOrientatiion+degree > 180):
    #         currOrientatiion = -90 - degree
    global currOrientation
    print(currOrientation)
    if currOrientation + degree < 360:
        currOrientation = currOrientation + degree
    if currOrientation + degree >= 360:
        currOrientation = abs(degree - (360 - currOrientation))
    print(currOrientation)

##
# Change the orientation to where Robomow is looking from the matrix perspective
# after that it has turned to the right
##
def changeOrRight(degree):
    global currOrientation
    print(currOrientation)
    if currOrientation - degree >= 0:
        currOrientation = currOrientation - degree
    if currOrientation - degree < 0:
        currOrientation = 360 - (degree - currOrientation)
    print(currOrientation)
