from models import create_model
import os
from opt import Options

class APDrawGAN:
    def __init__(self, base_path):
        """
        Initialize Test Setup of APDrawGAN
        """
        print(os.getcwd())
        model_path = os.path.join(base_path, 'checkpoints/formal_author/300_net_gen.pt')
        self.opt = Options(model_path)
        self.model = create_model(self.opt)
        self.model.setup(self.opt)
    
    def run(self, data):
        """
        Run APDrawGAN
        :param data: input data according to Authors, create it using APDRAW_GAN module
        :return: network output sketch
        """
        self.model.set_input(data)
        self.model.test()
        
        return self.model.fake_B[0,0,:,:].numpy()