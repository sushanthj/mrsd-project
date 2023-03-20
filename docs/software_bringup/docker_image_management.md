---
layout: default
title: Docker and Dockerhub
parent: Systems Setup
nav_order: 4
---

<details open markdown="block">
  <summary>
    Table of contents
  </summary>
  {: .text-delta }
1. TOC
{:toc}
</details>

# Create Docker Image from Active Container

1. run ```docker ps -a``` to display the active containers and note the name of container of interest (eg. mfi_container)
2. ```docker commit mfi_container```
3. Now if you run ```docker images```, you will see a new image which has no name. Let's give it a name
4. Name the new image using the Image ID of the unnamed container. Do ```docker tag 03423523 humble_sim_built```
5. Now if you run ```docker images```, you should see your new image

# Upload Image to DockerHub

1. Tag your image of interest with your dockerhub username: ```docker tag 03423523 sushanthj/humble_sim_built```
2. ```docker push sushanthj/humble_sim_built```

# Download Docker Image

```bash
docker pull sushanthj/humble_sim_built
```
