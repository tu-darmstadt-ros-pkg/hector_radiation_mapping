from typing import List

from gp import GPModel
from sample import Sample
from utils import split_tensor
import torch


class GPManager:
    def __init__(self, dimension):
        """
        Initialize gp model
        :param dimension: dimension of gp model
        """
        self.gp_model_for_use = GPModel()
        self.gp_model_update = GPModel()
        self.dimension = dimension
        self.new_model_available = False

    def add_samples(self, samples: List[Sample], params):
        """
        Add samples to gp model and train
        :param samples: list of samples
        :return: None
        """
        if not len(samples) == 0:
            # reformat samples into tensors
            x_star = torch.tensor([sample.position for sample in samples])
            x_star = x_star[:, :self.dimension]
            t = torch.tensor([sample.doseRate for sample in samples])
            print(x_star.shape, t.shape)
            self.gp_model_update.add_data(x_star, t)

        # add samples to gp model and train
        try:
            self.gp_model_update.train(params, len(samples)>0)
            self.new_model_available = True
        except():
            print("GP model " + str(self.dimension) + "D training failed")
            pass

    def evaluate(self, positions: torch.Tensor):
        """
        Evaluate gp model at positions and return mean and variance tensors of same length
        :param positions: tensor of positions
        :return: mean and variance tensors
        """
        if not self.gp_model_update.isTrained:
            print("GP model " + str(self.dimension) + "D not trained")
            return torch.empty(0, 1), torch.empty(0, 1)

        #self.update_to_newest_model()

        # cut positions into portions of 10000
        num_rows = positions.size(0)
        eval_batches = split_tensor(positions, 10000)
        mean, variance = torch.empty(num_rows, 1), torch.empty(num_rows, 1)

        # evaluate each batch and concatenate results
        start = 0
        for batch in eval_batches:
            end = start + batch.size(0)
            mean[start:end, :], variance[start:end, :] = self.gp_model_update.evaluate(batch)
            start = end

        print(mean.shape, variance.shape)
        return mean, variance

    def update_to_newest_model(self):
        """
        Update gp model to newest version
        :return: None
        """
        if self.new_model_available:
            self.gp_model_for_use = self.gp_model_update.copy()
            self.new_model_available = False