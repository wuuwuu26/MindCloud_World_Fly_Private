import torch as th
import torch.nn as nn
from config.config import cfg


class HeightLoss(nn.Module):
    def __init__(self, L):
        super(HeightLoss, self).__init__()
        self._L = L
        self.device = L.device
        self.sgm_time = cfg["sgm_time"]
        self.fixed_height = cfg["fixed_height"]
        self.eval_points = cfg["height_loss_eval_points"]

    def forward(self, Df, Dp):
        """
        Penalize future trajectory samples that leave the fixed-height plane.

        The sampled start states still span the original z-range, so the current point at t=0
        cannot be forced onto the plane. We therefore evaluate only future samples and place a
        larger weight on later points in the segment.
        """
        batch_size = Dp.shape[0]
        L = self._L.unsqueeze(0).expand(batch_size, -1, -1)
        coe = self.get_coefficient_from_derivative(Dp, Df, L)

        dt = self.sgm_time / self.eval_points
        t_list = th.linspace(dt, self.sgm_time, self.eval_points, device=self.device)
        t_list = t_list.view(1, -1, 1).expand(batch_size, -1, -1)

        z_samples = self.get_height_from_coeff(coe, t_list)
        height_error = z_samples - self.fixed_height

        sample_weight = (t_list.squeeze(-1) / self.sgm_time) ** 2
        sample_weight = sample_weight / sample_weight.mean(dim=1, keepdim=True)
        return (height_error.square() * sample_weight).mean(dim=1)

    def get_coefficient_from_derivative(self, Dp, Df, L):
        coefficient = th.zeros(Dp.shape[0], 18, device=self.device, dtype=Dp.dtype)

        for i in range(3):
            d = th.cat([Df[:, i, :], Dp[:, i, :]], dim=1).unsqueeze(-1)
            coefficient[:, 6 * i: 6 * (i + 1)] = (L @ d).squeeze(-1)

        return coefficient

    def get_height_from_coeff(self, coe, t):
        t_power = th.stack([th.ones_like(t), t, t ** 2, t ** 3, t ** 4, t ** 5], dim=-1).squeeze(-2)
        coe_z = coe[:, 12:18]
        return th.sum(t_power * coe_z.unsqueeze(1), dim=-1)
