FROM mit6832/drake-course:drake-20180205

RUN pip install meshcat==0.0.13
RUN rm -rf /drake
COPY drake/ /drake/
COPY jupyter-lab /bin/jupyter-lab
RUN chmod +x /bin/jupyter-lab

ARG NB_USER=jovyan
ARG NB_UID=1000
ENV USER ${NB_USER}
ENV NB_UID ${NB_UID}
ENV HOME /home/${NB_USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${NB_UID} \
    ${NB_USER}

COPY ./disr_lab ${HOME}/disr_lab
USER root
RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}
