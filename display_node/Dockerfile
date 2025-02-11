FROM ros:humble-perception

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-opencv \
    python3-pip \
    python3-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip install opencv-python

RUN pip install --upgrade pip && \
    pip install \
    "numpy>=1.21.0" \
    "opencv-python>=4.5.0"


WORKDIR /app

COPY display_node/ /app/display_node/


# Create an entrypoint script
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nexec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set the entry point
ENTRYPOINT ["/entrypoint.sh"]

CMD ["python3", "/app/display_node/main.py"]