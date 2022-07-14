import os
import xml.etree.ElementTree
import numpy as np


def xml_2_rois(xml_file_path):
    if not os.path.isfile(xml_file_path):
        return None
    pts_xml = xml.etree.ElementTree.parse(xml_file_path)
    root = pts_xml.getroot()
    vectors_xml = root
    rois = []
    for i_roi in range(len(vectors_xml)):
        vector_dict = vectors_xml[i_roi]
        pts = []
        for i_pt in range(len(vector_dict)):
            pt_dict = vector_dict[i_pt].attrib
            pts.append([float(pt_dict['x']), float(pt_dict['y'])])
        rois.append(pts)
    return rois


def rois_2_xml(rois, xml_file_path):
    print('writing to ', xml_file_path)
    # if rois is None or len(rois) == 0:
    #     if os.path.isfile(xml_file_path):
    #         os.remove(xml_file_path)
    #     return True
    if isinstance(rois, np.ndarray):
        rois = rois.tolist()
    vectors_xml = xml.etree.ElementTree.Element('vectors')
    num_pt = 0
    for i_roi, roi in enumerate(rois):
        vector_tag = 'vector_'+str(i_roi)
        pts = roi
        if len(pts) > 0:
            vector_xml = xml.etree.ElementTree.SubElement(vectors_xml, vector_tag)
            for i_pt, pt in enumerate(pts):
                assert len(pt) == 2
                pt_tag = 'point_'+str(num_pt)
                pt_xml = {'x': '{:.6f}'.format(pt[0]), 'y': '{:.6f}'.format(pt[1])}
                xml.etree.ElementTree.SubElement(vector_xml, pt_tag, attrib=pt_xml)
                num_pt += 1

    xml_file = xml.etree.ElementTree.tostring(vectors_xml)

    if os.path.isfile(xml_file_path):
        pass
    else:
        pass
    with open(xml_file_path, 'wb') as f:
        f.write(xml_file)
    print(xml_file_path, 'written ')
    return True


def xml_2_pts(xml_file_path):
    if not os.path.isfile(xml_file_path):
        return None
    pts_xml = xml.etree.ElementTree.parse(xml_file_path)
    root = pts_xml.getroot()
    pts = []
    for i in range(len(root)):
        pt_dict = root[i].attrib
        pts.append([float(pt_dict['x']), float(pt_dict['y'])])
    return pts


def pts_2_xml(pts, xml_file_path):
    # if not pts or len(pts) == 0:
    #     if os.path.isfile(xml_file_path):
    #         os.remove(xml_file_path)
    #     return True
    if isinstance(pts, np.ndarray):
        pts = pts.tolist()
    vector_xml = xml.etree.ElementTree.Element('vector')
    for i in range(len(pts)):
        pt = pts[i]
        if len(pt) == 2:
            pt_tag = 'point_'+str(i)
            pt_xml = {'x': '{:.6f}'.format(pt[0]), 'y': '{:.6f}'.format(pt[1])}
            xml.etree.ElementTree.SubElement(vector_xml, pt_tag, attrib=pt_xml)
    xml_file = xml.etree.ElementTree.tostring(vector_xml)

    if os.path.isfile(xml_file_path):
        pass
    else:
        pass
    with open(xml_file_path, 'wb') as f:
        f.write(xml_file)
    return True


def xml_2_ids(xml_file_path):
    if not os.path.isfile(xml_file_path):
        return None
    pts_xml = xml.etree.ElementTree.parse(xml_file_path)
    root = pts_xml.getroot()
    ids = []
    for i in range(len(root)):
        pt_dict = root[i].attrib
        ids.append(int(pt_dict['index']))
    return ids


def ids_2_xml(indexes, xml_file_path):
    # if not pts or len(pts) == 0:
    #     if os.path.isfile(xml_file_path):
    #         os.remove(xml_file_path)
    #     return True
    if isinstance(indexes, np.ndarray):
        indexes = indexes.tolist()
    vector_xml = xml.etree.ElementTree.Element('vector')
    for i in range(len(indexes)):
        index = indexes[i]
        pt_tag = 'point_'+str(i)
        pt_xml = {'index': str(index)}
        xml.etree.ElementTree.SubElement(vector_xml, pt_tag, attrib=pt_xml)
    xml_file = xml.etree.ElementTree.tostring(vector_xml)

    if os.path.isfile(xml_file_path):
        pass
    else:
        pass
    with open(xml_file_path, 'wb') as f:
        f.write(xml_file)
    return True


def test_xml_convertor():
    xml_file_path = r'D:/proj/hair_slam//data/vectors.xml'
    # xml_file_path = r'D:/Algorithm/Xml/Track/trackLeft.xml'
    ids = np.arange(0, 6)
    ids = [i for i in range(10)]
    print(ids)
    print(type(ids[0]))

    # roi = np.concatenate([pts, pts], axis=-1)
    # rois = [roi] * 3
    # print(rois[0].shape)
    ids_2_xml(ids, xml_file_path)
    ids = xml_2_ids(xml_file_path)

    print(ids)
    print(type(ids[0]))

    # for pt in pts:
    #     print(pt)

    # pts = xml_2_pts(xml_file_path)
    # pts_2_xml(pts, '/home/cheng/Downloads/Vectors_Cheng.xml')
    # test_timer()


def main():
    test_xml_convertor()


if __name__ == '__main__':
    main()
