import rospkg
import torch


def save_list_to_file(list, folder: str, file_name: str):
    """
    Saves a list to a file in the export folder with the given name and folder (folder must exist)
    :param list:  list to save
    :param folder:  folder name
    :param file_name:  file name
    :return: None
    """
    export_path = rospkg.RosPack().get_path('hector_radiation_mapping') + '/radiation_mapping_exports/'
    with open(export_path + folder + "/" + file_name, 'w') as f:
        for item in list:
            f.write(str(item) + "\n")


def split_tensor(tensor, max_size):
    """
    Split tensor into chunks of max_size along first dimension and return list of chunks
    :param tensor: tensor to split
    :param max_size: maximum size of chunks
    :return: list of chunks
    """
    num_chunks = (tensor.size(0) + max_size - 1) // max_size
    chunks = torch.chunk(tensor, num_chunks, dim=0)
    return chunks
