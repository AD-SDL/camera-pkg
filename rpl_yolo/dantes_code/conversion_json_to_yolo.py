import json
pic_height = 720
pic_width = 720

#/home/hdantes/datasets/OT_2/Datasetbf95119c-044c-4729-ae63-c3abbfc252c7
#/home/hdantes/datasets/OT_2/Datasetbc59311b-39d0-4060-8452-5b958f62d7f7
for i in range(7):
    with open("/home/hdantes/datasets/OT_2/Datasetbc59311b-39d0-4060-8452-5b958f62d7f7/captures_00"+str(i)+".json", 'r') as file:
        big_json_file = json.load(file)

    for picture in big_json_file['captures']:
        filename = picture['filename'].split('/')[-1]
        filename = filename[:-4] + '.txt'

        with open(filename, 'w') as annotation_file:
            for bbox in picture['annotations'][0]['values']:
                annotation_file.write(
                    '%d %f %f %f %f\n' % (
                        int(bbox['label_id'])-1,
                        (bbox['x'] + bbox['width'] /2)  / pic_width,
                        (bbox['y'] + bbox['height']/2)  / pic_height,
                        bbox['width']                   / pic_width,
                        bbox['height']                  / pic_height
                        )
                    )
