FROM ros2
RUN apt update
RUN git -C /root clone https://github.com/NVlabs/Fast-FoundationStereo.git
RUN pip install torch==2.6.0 torchvision==0.21.0 xformers --index-url https://download.pytorch.org/whl/cu124 --break-system-packages
RUN pip install timm --break-system-packages
RUN pip install einops --break-system-packages
RUN pip install omegaconf --break-system-packages
RUN pip install scipy --break-system-packages
RUN pip install numpy --break-system-packages
RUN pip install scikit-image --break-system-packages
RUN pip install imageio --break-system-packages
RUN pip install pyyaml --break-system-packages
RUN pip install open3d --break-system-packages
ENV PYTHONPATH $PYTHONPATH:/root/Fast-FoundationStereo
COPY /20-30-48 /root/Fast-FoundationStereo/weights/20-30-48
RUN mkdir src
RUN git -C src clone https://github.com/jfrancis71/ros2_lego
CMD /bin/bash -c "python3 src/ros2_lego/ffs/ros2.py"

#docker run -it --rm --privileged --network=host --ipc=host -v ros2_ws:/root/ros2_ws -v $HOME/.gitconfig:/root/.gitconfig -v $HOME/.git-credentials:/root/.git-credentials     -v="$XAUTHORITY:$XAUTHORITY" --env="XAUTHORITY=$XAUTHORITY"  --env="DISPLAY=$DISPLAY" -v ~/depth_models/Fast-FoundationStereo/weights:/root/Fast-FoundationStereo/weights --gpus all ros2_ffs /bin/bash
