''' Most of the code here will use the conversion script (the script that converts the json files into YOLO readable
annnotation text files'''
import os
files = os.listdir('.')
print(files)
path = input("Give the path:")
import json
pic_height = 720
pic_width = 720
with open('/home/hdantes/datasets/attempt2/images/captures_000.json','r') as file:
        big_json_file = json.load(file)
for picture in big_json_file['captures']:
    filename = picture['filename'].split('/')[-1]
    filename = filename[:-4]+'.txt'

with open(filename,'w') as annotation_file:
    for bbox in picture['annotations'][0]['values']:
        annotation_file.write(
            '%d %f %f %f %f\n' % (
                    # Since label_id starts from 1 and yolo starts from 0, this just fixes that. If not fixed, error regarding the number of classes will occur
                    int(bbox['label_id'])-1,
                    (bbox['x'] + bbox['width'] /2)  / pic_width,
                    (bbox['y'] + bbox['height']/2)  / pic_height,
                    bbox['width']                   / pic_width,
                    bbox['height']                  / pic_height
                    )
        )
max = 0
for bbox in picture['annotations'][0]['values']:
    if max <= int(bbox['label_id']):
        max = int(bbox['label_id'])
nc = max
names = []
for i in range(max):
     names.append(i)
for picture in big_json_file['captures']:
    with open(filename, 'w') as annotation_file:
        for bbox in picture['annotations'][0]['values']:
            names[int(bbox['label_id'])-1] = bbox['label_name']
#print(names)
path_string =''
# '\nnames: [' + space.join('\''+str(e)+ '\',' for e in names)+ ']'
# for e in names:
#     if names.index(e) == len(names)-1:
#         path_string.join(str(e))
#     else:
#         path_string.join(' '+str(e)+',')
for e in names:
    if names.index(e)== 0:
        path_string += '['+e+', '
    elif names.index(e) == len(names)-1:
        path_string += e+']'
    
    else:
        path_string += e + ', '
train = 'images'
val = 'images'
test = ''
print(path_string)
l='path: '+ path+ '\ntrain: '+ train + '\nval: '+ val+ '\ntest: '+ test +'\nnc: '+ str(max)+ '\nnames: '+path_string
# filename = 'dantes_test.yaml'

# with open(filename, 'w') as annotation_file:
#     annotation_file.write(l)


#l='path: '+ path+ '\ntrain: '+ train + '\nval: '+ val+ '\ntest: '+ path_string
filename = 'dantes_test_2.yaml'

with open(filename, 'w') as annotation_file:
    annotation_file.write(l)
            
