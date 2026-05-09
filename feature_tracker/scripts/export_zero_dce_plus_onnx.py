#!/usr/bin/env python
from __future__ import print_function

import argparse
import os

import torch
import torch.nn as nn
import torch.nn.functional as F


class DepthwiseSeparableConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(DepthwiseSeparableConv, self).__init__()
        self.depth_conv = nn.Conv2d(in_channels, in_channels, 3, 1, 1, groups=in_channels, bias=True)
        self.point_conv = nn.Conv2d(in_channels, out_channels, 1, 1, 0, bias=True)

    def forward(self, x):
        return self.point_conv(self.depth_conv(x))


class ZeroDcePlusNet(nn.Module):
    def __init__(self, scale_factor=12):
        super(ZeroDcePlusNet, self).__init__()
        number_f = 32
        self.relu = nn.ReLU(inplace=True)
        self.scale_factor = max(1, int(scale_factor))
        self.e_conv1 = DepthwiseSeparableConv(3, number_f)
        self.e_conv2 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv3 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv4 = DepthwiseSeparableConv(number_f, number_f)
        self.e_conv5 = DepthwiseSeparableConv(number_f * 2, number_f)
        self.e_conv6 = DepthwiseSeparableConv(number_f * 2, number_f)
        self.e_conv7 = DepthwiseSeparableConv(number_f * 2, 3)

    def forward(self, x):
        if self.scale_factor == 1:
            x_down = x
        else:
            height = max(1, int(x.size(2)) // self.scale_factor)
            width = max(1, int(x.size(3)) // self.scale_factor)
            x_down = F.interpolate(x, size=(height, width), mode="bilinear", align_corners=False)

        x1 = self.relu(self.e_conv1(x_down))
        x2 = self.relu(self.e_conv2(x1))
        x3 = self.relu(self.e_conv3(x2))
        x4 = self.relu(self.e_conv4(x3))
        x5 = self.relu(self.e_conv5(torch.cat([x3, x4], 1)))
        x6 = self.relu(self.e_conv6(torch.cat([x2, x5], 1)))
        x_r = torch.tanh(self.e_conv7(torch.cat([x1, x6], 1)))
        if self.scale_factor != 1:
            x_r = F.interpolate(x_r, size=(int(x.size(2)), int(x.size(3))), mode="bilinear", align_corners=False)

        for _ in range(8):
            x = x + x_r * (torch.pow(x, 2) - x)
        return x


def parse_args():
    parser = argparse.ArgumentParser(description="Export Zero-DCE++ to ONNX.")
    parser.add_argument("--weights", default="feature_tracker/models/zero_dce_plus_epoch99.pth")
    parser.add_argument("--output", default="feature_tracker/models/zero_dce_plus_480x640_sf12.onnx")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--scale-factor", type=int, default=12)
    parser.add_argument("--opset", type=int, default=11)
    return parser.parse_args()


def main():
    args = parse_args()
    model = ZeroDcePlusNet(args.scale_factor)
    state_dict = torch.load(args.weights, map_location="cpu")
    model.load_state_dict(state_dict)
    model.eval()

    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    dummy = torch.zeros(1, 3, args.height, args.width, dtype=torch.float32)
    export_kwargs = dict(
        export_params=True,
        opset_version=args.opset,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
    )
    with torch.no_grad():
        try:
            torch.onnx.export(model, dummy, args.output, dynamo=False, external_data=False, **export_kwargs)
        except TypeError:
            torch.onnx.export(model, dummy, args.output, **export_kwargs)
    print("exported:", args.output)


if __name__ == "__main__":
    main()
