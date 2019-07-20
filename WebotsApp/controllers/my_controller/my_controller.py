from controller import Supervisor

supervisor = Supervisor()

robotNode = supervisor.getFromDef('Burger')

translationField = robotNode.getField('translation')

freq = 0

while supervisor.step(32) != -1:
    if freq == 10:
        translation = translationField.getSFVec3f()
        print('Yellow position: %g %g %g\n' % (translation[0], translation[1], translation[2]))
        freq = 0
    freq += 1