import glob
import os
import shutil
import argparse

FLAGS = None

#classes = ['red', 'green', 'yellow']
# just RED / NOT RED
classes = ['red']
images = {}

def prepare_images():

    pattern = os.path.join(FLAGS.input_dir, '*{}.jpg')
    all_classes = set()

    for c in classes:
        images[c] = glob.glob(pattern.format(c.upper()))
        all_classes |= set(images[c])
        print(c + ' images:', len(images[c]))

    all_images = set(glob.glob(pattern.format('')))
    unknown = all_images - set(all_classes)
    images['unknown'] = list(unknown)

    directory = FLAGS.output_dir

    if not os.path.exists(directory):
        os.makedirs(directory)

    for c in images.keys():
        class_dir = os.path.join(directory, c)
        if not os.path.exists(class_dir):
            os.makedirs(class_dir)
        
        if len(os.listdir(class_dir)) == 0:
            for i in images[c]:
                shutil.copy(i, class_dir)
        else:
            print('WARN: directory {} is not empty.'.format(class_dir))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--input_dir',
        type=str,
        default='',
        help='Path to image folder.'
    )

    parser.add_argument(
        '--output_dir',
        type=str,
        default='/tmp/images',
        help='Path to folder of labeled images.'
    )

    FLAGS, unparsed = parser.parse_known_args()
    prepare_images()