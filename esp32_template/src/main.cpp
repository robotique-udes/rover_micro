#include <micro_ros_esp32.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

// Définition du topic
static const char *TOPIC_NAME = "compteur";

// Structure du message
typedef struct
{
    int32_t count;
} CompteurMsg;

// Variables globales
rcl_publisher_t publisher;
CompteurMsg msg;

// Fonction de publication
void publish_count()
{
    msg.count++;
    rcl_publish(&publisher, &msg, NULL);
}

void setup()
{
    // Initialisation du micro-ROS
    micro_ros_esp32_init();

    // Création du noeud
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    rcl_ret_t ret = rcl_node_init(&node, "compteur_node", &node_options);
    if (ret != RCL_RET_OK)
    {
        rcl_error("Echec de l'initialisation du noeud : %d", ret);
        return;
    }

    // Création du topic
    publisher = rcl_publish_init(&node, TOPIC_NAME, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32));
    if (!publisher)
    {
        rcl_error("Echec de la création du topic : %d", ret);
        return;
    }

    // Configuration du timer
    esp_timer_create_args_t timer_args = {
        .callback = &publish_count,
        .arg = NULL,
        .name = "compteur_timer"};
    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, 1000000); // 1 seconde
}

void loop()
{
    // Traitement des messages reçus
    rcl_wait_for_messages(NULL);
}
