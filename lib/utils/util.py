import os, pygame
#get the abspath of current script
_ME_PATH = os.path.abspath(os.path.dirname(__file__))
DATA_PATH = os.path.normpath(os.path.join(_ME_PATH, '../../..', 'data'))
def file_path(filename=None):
    """ give a file(img, sound, font...) name, return full path name. """
    if filename is None:
        raise ValueError('must supply a filename')
    # get filename extension
    fileext = os.path.splitext(filename)[1]
    if fileext in ('.png', '.bmp', '.tga', '.jpg'):
        sub = 'image'
    elif fileext in ('.ogg', '.mp3', '.wav'):
        sub = 'sound'
    elif fileext in ('.ttf',):
        sub = 'font'

    file_path = os.path.join(DATA_PATH, sub, filename)
    print('Will read', file_path)

    if os.path.abspath(file_path):
        return file_path
    else:
        raise ValueError("Cant open file `%s'." % file_path)

def scale_image(img, factor):
    size = round(img.get_width() * factor), round(img.get_height() * factor)
    return pygame.transform.scale(img, size)