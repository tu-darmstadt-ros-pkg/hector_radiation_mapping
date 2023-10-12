import torch
import gpytorch


class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, data_x, data_y, likelihood):
        """
        Initialize gp model with data and likelihood function
        :param data_x:  tensor of x values
        :param data_y:  tensor of y values
        :param likelihood:  likelihood function
        """
        super(ExactGPModel, self).__init__(data_x, data_y, likelihood)
        self.mean_module = gpytorch.means.ConstantMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

    def forward(self, x):
        """
        Forward pass of gp model with x
        :param x: tensor of x values
        :return: MultivariateNormal
        """
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)


class GPModel:
    def __init__(self):
        """
        Initialize gp model
        """
        self.isTrained = None
        self.data_x = None
        self.data_y = None
        self.parameters = [1.0, 1.0, 1.0]
        self.model = ExactGPModel(self.data_x, self.data_y, gpytorch.likelihoods.GaussianLikelihood())
        #self.cuda_available = torch.cuda.is_available()
        self.cuda_available = False


    def add_data(self, x_star_, y_):
        """
        Add data to gp model
        :param x_star_: tensor of x values
        :param y_: tensor of y values
        :return: None
        """
        print("Adding data", x_star_.shape, y_.shape)
        if self.data_x is None:
            self.data_x = x_star_
            self.data_y = y_
        else:
            self.data_x = torch.cat((self.data_x, x_star_), 0)
            self.data_y = torch.cat((self.data_y, y_), 0)
        if self.cuda_available:
            self.data_x = self.data_x.cuda()
            self.data_y = self.data_y.cuda()
        print(self.data_x.shape, self.data_y.shape)

    def train(self, params, newSamples, num_iterations=2, print_iterations=True):
        """
        Train gp model
        :param num_iterations: number of maximum iterations
        :param print_iterations: print iterations
        :return: None
        """
        self.parameters = params
        print("Trained GP model", self.parameters)
        #print("Training GP model", self.data_x.shape, self.data_y.shape)
        #self.model.covar_module.base_kernel.lengthscale = self.parameters[0]
        #self.model.covar_module.outputscale = self.parameters[1]
        #self.model.likelihood.noise = self.parameters[2]

        self.model = ExactGPModel(self.data_x, self.data_y, gpytorch.likelihoods.GaussianLikelihood())
        self.model.covar_module.base_kernel.lengthscale = self.parameters[0]
        self.model.covar_module.outputscale = self.parameters[1]
        self.model.likelihood.noise = self.parameters[2]
        print(self.model.parameters())
        print(self.model.named_parameters())
        if not newSamples:
            return

        self.model.set_train_data(self.data_x, self.data_y, strict=False)

        train = False
        if train:
            self.model.train()
            self.model.likelihood.train()
            optimizer = torch.optim.Adam(self.model.parameters(), lr=0.1)
            mll = gpytorch.mlls.ExactMarginalLogLikelihood(self.model.likelihood, self.model)
            for iteration in range(num_iterations):
                optimizer.zero_grad()
                output = self.model(self.data_x)
                loss = -mll(output, self.data_y)
                loss.backward()

                if print_iterations:
                    print('Iter %d/%d - Loss: %.3f   lengthscale: %.3f   noise: %.3f' % (
                        iteration + 1, num_iterations, loss.item(),
                        self.model.covar_module.base_kernel.lengthscale.item(),
                        self.model.likelihood.noise.item()
                    ))
                optimizer.step()

        self.parameters[0] = self.model.covar_module.base_kernel.lengthscale
        self.parameters[1] = self.model.covar_module.outputscale
        self.parameters[2] = self.model.likelihood.noise
        print("Trained GP model", self.parameters)
        self.isTrained = True

    def evaluate(self, eval_points):
        """
        Evaluate gp model with eval_points and return mean and variance of prediction distribution
        :param eval_points: tensor of x values to evaluate model on
        :return: mean and variance of prediction distribution
        """
        eval_torch = eval_points
        print("Evaluating GP model", eval_torch.shape, self.data_x.shape, self.data_y.shape)
        self.model.eval()
        self.model.likelihood.eval()

        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            res = self.model(eval_torch)
            predictions = self.model.likelihood(res)

        return predictions.mean.detach().cpu().view(-1, 1), predictions.variance.detach().cpu().view(-1, 1)
