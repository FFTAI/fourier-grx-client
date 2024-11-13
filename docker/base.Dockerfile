ARG UBUNTU_VERSION=20.04
FROM 192.168.3.15:9595/mirror/ubuntu:${UBUNTU_VERSION}

ARG PYTHON_VERSION=3.11
ARG DEBIAN_FRONTEND=noninteractive

# Add deadsnakes PPA for Python 3.10
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common gpg-agent \
    && add-apt-repository 'ppa:deadsnakes/ppa' \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # build-essential cmake gcc \
    python${PYTHON_VERSION}-dev python${PYTHON_VERSION}-venv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create virtual environment
RUN ln -s /usr/bin/python${PYTHON_VERSION} /usr/bin/python
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN echo "source /opt/venv/bin/activate" >> /root/.bashrc
