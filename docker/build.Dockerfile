FROM yuxianggao/python:3.11-20.04

COPY . /fourier_grx_client
WORKDIR /fourier_grx_client
RUN pip install --upgrade --no-cache-dir pip
RUN pip install pdm

# build the project with pdm
RUN pdm build
